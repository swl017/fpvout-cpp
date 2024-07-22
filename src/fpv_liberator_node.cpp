#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <libusb.h>
#include <err.h>
#include <sys/timeb.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <atomic>
#include <signal.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

#define INBUF_SIZE 4096
#define SHM_NAME "/fpv_liberator_shm"
#define SHM_SIZE 1048576  // 1MB shared memory size
#define FRAME_WIDTH 640   // Adjust these values according to your video format
#define FRAME_HEIGHT 480
#define MAX_RETRIES 5

class FPVLiberator {
public:
    FPVLiberator(ros::NodeHandle& nh);
    ~FPVLiberator();
    void run();

private:
    struct DecoderContext {
        const AVCodec *codec;
        AVCodecParserContext *parser;
        AVCodecContext *c;
        AVFrame *frame;
        AVPacket *pkt;
        SwsContext *sws_ctx;
    };

    ros::NodeHandle& nh_;
    ros::Publisher image_pub_;
    libusb_device_handle* dh_;
    struct libusb_transfer *xfr_;
    DecoderContext* decoder_ctx_;
    void* shm_ptr_;
    int shm_fd_;
    std::vector<uint8_t> buf_;
    std::atomic<bool> should_run_;

    double last_;
    int bytes_;
    size_t shm_offset_;
    cv::Mat frame_;
    std::chrono::steady_clock::time_point last_packet_;

    DecoderContext* init_decoder();
    void close_decoder(DecoderContext* ctx);
    void decode(DecoderContext* ctx, uint8_t* data, int data_size);
    double now();
    libusb_device_handle* open_device();
    void close_device();
    static void transfer_cb(struct libusb_transfer *xfer);
    void handle_transfer(struct libusb_transfer *xfer);
    void cleanup();
};

FPVLiberator::FPVLiberator(ros::NodeHandle& nh) 
    : nh_(nh), dh_(nullptr), xfr_(nullptr), decoder_ctx_(nullptr), shm_ptr_(nullptr), shm_fd_(-1),
      buf_(65536), should_run_(true), last_(0), bytes_(0), shm_offset_(0),
      frame_(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3), last_packet_(std::chrono::steady_clock::now()) {
    
    image_pub_ = nh_.advertise<sensor_msgs::Image>("fpv_image", 1);

    ROS_INFO("Initializing libusb");
    if (libusb_init(NULL) < 0) {
        ROS_ERROR("Failed to initialize libusb");
        ros::shutdown();
        return;
    }

    ROS_INFO("Creating shared memory");
    shm_fd_ = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd_ == -1) {
        ROS_ERROR("Failed to create shared memory: %s", strerror(errno));
        ros::shutdown();
        return;
    }

    if (ftruncate(shm_fd_, SHM_SIZE) == -1) {
        ROS_ERROR("Failed to configure shared memory size: %s", strerror(errno));
        cleanup();
        ros::shutdown();
        return;
    }

    shm_ptr_ = mmap(0, SHM_SIZE, PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (shm_ptr_ == MAP_FAILED) {
        ROS_ERROR("Failed to map shared memory: %s", strerror(errno));
        cleanup();
        ros::shutdown();
        return;
    }

    decoder_ctx_ = init_decoder();
    if (!decoder_ctx_) {
        ROS_ERROR("Failed to initialize decoder");
        cleanup();
        ros::shutdown();
        return;
    }

    last_ = now();
}

FPVLiberator::~FPVLiberator() {
    cleanup();
}

void FPVLiberator::cleanup() {
    ROS_INFO("Cleaning up resources");
    if (dh_) close_device();
    if (decoder_ctx_) close_decoder(decoder_ctx_);
    if (shm_ptr_ != MAP_FAILED) munmap(shm_ptr_, SHM_SIZE);
    if (shm_fd_ != -1) {
        close(shm_fd_);
        shm_unlink(SHM_NAME);
    }
    libusb_exit(NULL);
}

void FPVLiberator::run() {
    while (ros::ok() && should_run_) {
        if (!dh_) {
            ROS_INFO("Opening device");
            dh_ = open_device();
            if (!dh_) {
                ROS_WARN("Failed to open device, retrying in 5 seconds...");
                ros::Duration(5.0).sleep();
                continue;
            }

            if (xfr_) libusb_free_transfer(xfr_);
            xfr_ = libusb_alloc_transfer(0);
            libusb_fill_bulk_transfer(xfr_, dh_, 0x84, buf_.data(), buf_.size(), transfer_cb, this, 1000);
            
            if (libusb_submit_transfer(xfr_) < 0) {
                ROS_ERROR("Failed to submit initial transfer");
                close_device();
                continue;
            }
        }

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        int completed = 0;
        if (libusb_handle_events_timeout_completed(NULL, &tv, &completed) != LIBUSB_SUCCESS) {
            ROS_ERROR("Error in event handling");
            close_device();
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_packet_).count() > 5) {
            ROS_WARN("No packets received for 5 seconds, reconnecting...");
            close_device();
            continue;
        }

        ros::spinOnce();
    }
}

FPVLiberator::DecoderContext* FPVLiberator::init_decoder() {
    DecoderContext* ctx = new DecoderContext();

    ctx->codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!ctx->codec) {
        ROS_ERROR("Codec not found");
        delete ctx;
        return nullptr;
    }

    ctx->parser = av_parser_init(ctx->codec->id);
    if (!ctx->parser) {
        ROS_ERROR("Parser not found");
        delete ctx;
        return nullptr;
    }

    ctx->c = avcodec_alloc_context3(ctx->codec);
    if (!ctx->c) {
        ROS_ERROR("Could not allocate video codec context");
        av_parser_close(ctx->parser);
        delete ctx;
        return nullptr;
    }

    if (avcodec_open2(ctx->c, ctx->codec, NULL) < 0) {
        ROS_ERROR("Could not open codec");
        avcodec_free_context(&ctx->c);
        av_parser_close(ctx->parser);
        delete ctx;
        return nullptr;
    }

    ctx->frame = av_frame_alloc();
    ctx->pkt = av_packet_alloc();
    ctx->sws_ctx = nullptr;

    return ctx;
}

void FPVLiberator::close_decoder(DecoderContext* ctx) {
    if (ctx) {
        av_parser_close(ctx->parser);
        avcodec_free_context(&ctx->c);
        av_frame_free(&ctx->frame);
        av_packet_free(&ctx->pkt);
        sws_freeContext(ctx->sws_ctx);
        delete ctx;
    }
}

void FPVLiberator::decode(DecoderContext* ctx, uint8_t* data, int data_size) {
    uint8_t *ptr = data;
    while (data_size > 0) {
        int ret = av_parser_parse2(ctx->parser, ctx->c, &ctx->pkt->data, &ctx->pkt->size,
                                   ptr, data_size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
        if (ret < 0) {
            ROS_ERROR("Error while parsing");
            return;
        }
        ptr += ret;
        data_size -= ret;

        if (ctx->pkt->size) {
            int send_ret = avcodec_send_packet(ctx->c, ctx->pkt);
            if (send_ret < 0) {
                ROS_ERROR("Error sending packet for decoding");
                return;
            }

            while (send_ret >= 0) {
                int receive_ret = avcodec_receive_frame(ctx->c, ctx->frame);
                if (receive_ret == AVERROR(EAGAIN) || receive_ret == AVERROR_EOF)
                    break;
                else if (receive_ret < 0) {
                    ROS_ERROR("Error during decoding");
                    return;
                }

                if (!ctx->sws_ctx) {
                    ctx->sws_ctx = sws_getContext(ctx->c->width, ctx->c->height, ctx->c->pix_fmt,
                                                  ctx->c->width, ctx->c->height, AV_PIX_FMT_BGR24,
                                                  SWS_BILINEAR, NULL, NULL, NULL);
                }

                cv::Mat frame(ctx->c->height, ctx->c->width, CV_8UC3);
                uint8_t *dest[4] = {frame.data, NULL, NULL, NULL};
                int dest_linesize[4] = {frame.step, 0, 0, 0};
                sws_scale(ctx->sws_ctx, ctx->frame->data, ctx->frame->linesize, 0, ctx->c->height, dest, dest_linesize);

                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
                image_pub_.publish(msg);
            }
        }
    }
}

double FPVLiberator::now() {
    struct timeb timebuffer;
    ftime(&timebuffer);
    return timebuffer.time + ((double)timebuffer.millitm)/1000;
}

libusb_device_handle* FPVLiberator::open_device() {
    libusb_device_handle *dh = libusb_open_device_with_vid_pid(NULL, 0x2ca3, 0x001f);
    if (dh) {
        if (libusb_claim_interface(dh, 3) < 0) {
            ROS_ERROR("Failed to claim interface");
            libusb_close(dh);
            return nullptr;
        }
        
        int tx = 0;
        uint8_t data[] = {0x52, 0x4d, 0x56, 0x54};
        if (libusb_bulk_transfer(dh, 0x03, data, 4, &tx, 500) < 0) {
            ROS_WARN("No data transmitted to device");
        }
    }
    return dh;
}

void FPVLiberator::close_device() {
    if (dh_) {
        libusb_release_interface(dh_, 3);
        libusb_close(dh_);
        dh_ = nullptr;
    }
}

void FPVLiberator::transfer_cb(struct libusb_transfer *xfer) {
    FPVLiberator* liberator = static_cast<FPVLiberator*>(xfer->user_data);
    liberator->handle_transfer(xfer);
}

void FPVLiberator::handle_transfer(struct libusb_transfer *xfer) {
    if (xfer->status != LIBUSB_TRANSFER_COMPLETED) {
        ROS_ERROR("Transfer failed: %s", libusb_error_name(xfer->status));
        return;
    }

    last_packet_ = std::chrono::steady_clock::now();

    decode(decoder_ctx_, xfer->buffer, xfer->actual_length);

    if (shm_offset_ + xfer->actual_length > SHM_SIZE) {
        shm_offset_ = 0;
    }
    memcpy((char*)shm_ptr_ + shm_offset_, xfer->buffer, xfer->actual_length);
    shm_offset_ += xfer->actual_length;

    if ((now() - last_) > (bytes_ ? 0.1 : 2)) {
        ROS_INFO("Receiving %f kb/s", (bytes_ / (now() - last_))/1000);
        last_ = now();
        bytes_ = 0;
    }
    bytes_ += xfer->actual_length;

    if (libusb_submit_transfer(xfer) < 0) {
        ROS_ERROR("Failed to resubmit transfer");
    }
}

void signal_handler(int signum) {
    ROS_INFO("Interrupt signal (%d) received.", signum);
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fpv_liberator");
    ros::NodeHandle nh;

    signal(SIGINT, signal_handler);

    FPVLiberator liberator(nh);
    liberator.run();

    return 0;
}