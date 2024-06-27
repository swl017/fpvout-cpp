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
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

using namespace std;
using namespace cv;

#define SHM_NAME "/fpv_liberator_shm"
#define SHM_SIZE 1048576  // 1MB shared memory size
#define FRAME_WIDTH 1280   // Adjust these values according to your video format
#define FRAME_HEIGHT 720
#define MAX_RETRIES 5
#define INBUF_SIZE 4096

class FPVLiberator {
private:
    struct DecoderContext {
        const AVCodec *codec;
        AVCodecParserContext *parser;
        AVCodecContext *c;
        AVFrame *frame;
        AVPacket *pkt;
        SwsContext *sws_ctx;
    };

    struct ctx {
        double last;
        int bytes;
        void* shm_ptr;
        size_t shm_offset;
        Mat frame;
        chrono::steady_clock::time_point last_packet;
        DecoderContext* decoder_ctx;  
    };

    ctx myctx;
    libusb_device_handle* dh;
    struct libusb_transfer *xfr;
    vector<uint8_t> buf;
    int shm_fd;

    // ROS-specific members
    ros::NodeHandle nh;
    ros::Publisher image_pub;

    DecoderContext* init_decoder();
    void decode(DecoderContext* ctx, uint8_t* data, int data_size);
    void close_decoder(DecoderContext* ctx);
    double now();
    int die(string msg, int code);
    libusb_device_handle* open_device();
    void close_device(libusb_device_handle* dh);
    static void transfer_cb(struct libusb_transfer *xfer);
    void publish_frame(const cv::Mat& frame);

public:
    FPVLiberator(int argc, char** argv);
    ~FPVLiberator();
    int run();
};

FPVLiberator::FPVLiberator(int argc, char** argv) : dh(NULL), xfr(NULL), buf(65536) {
    // Initialize ROS
    ros::init(argc, argv, "fpv_liberator");
    image_pub = nh.advertise<sensor_msgs::Image>("fpv_image", 1);

    cerr << "Using libusb v" << libusb_get_version()->major << "." << libusb_get_version()->minor << "." << libusb_get_version()->micro << endl;
    int err = 0;
    cerr << "initializing" << endl;
    if ((err = libusb_init(NULL)) < 0)
        die("initialize fail ", err);

    shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        die("Failed to create shared memory ", errno);
    }

    if (ftruncate(shm_fd, SHM_SIZE) == -1) {
        die("Failed to configure shared memory size ", errno);
    }

    void* shm_ptr = mmap(0, SHM_SIZE, PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED) {
        die("Failed to map shared memory ", errno);
    }

    namedWindow("Debug Output", WINDOW_NORMAL);

    DecoderContext* decoder_ctx = init_decoder();
    if (!decoder_ctx) {
        die("Failed to initialize decoder", -1);
    }

    myctx = {
        now(),
        0,
        shm_ptr,
        0,
        Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3),
        chrono::steady_clock::now(),
        decoder_ctx
    };
}

FPVLiberator::~FPVLiberator() {
    cerr << "closing" << endl;
    if (dh) close_device(dh);
    libusb_exit(NULL);

    munmap(myctx.shm_ptr, SHM_SIZE);
    close(shm_fd);
    shm_unlink(SHM_NAME);

    destroyAllWindows();
    close_decoder(myctx.decoder_ctx);
}

// ... (other methods remain the same)

void FPVLiberator::decode(DecoderContext* ctx, uint8_t* data, int data_size) {
    uint8_t *ptr = data;
    while (data_size > 0) {
        int ret = av_parser_parse2(ctx->parser, ctx->c, &ctx->pkt->data, &ctx->pkt->size,
                                   ptr, data_size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
        if (ret < 0) {
            std::cerr << "Error while parsing" << std::endl;
            return;
        }
        ptr += ret;
        data_size -= ret;

        if (ctx->pkt->size) {
            int send_ret = avcodec_send_packet(ctx->c, ctx->pkt);
            if (send_ret < 0) {
                std::cerr << "Error sending packet for decoding" << std::endl;
                return;
            }

            while (send_ret >= 0) {
                int receive_ret = avcodec_receive_frame(ctx->c, ctx->frame);
                if (receive_ret == AVERROR(EAGAIN) || receive_ret == AVERROR_EOF)
                    break;
                else if (receive_ret < 0) {
                    std::cerr << "Error during decoding" << std::endl;
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

                cv::imshow("Decoded Frame", frame);
                cv::waitKey(1);

                // Publish the frame as a ROS message
                publish_frame(frame);
            }
        }
    }
}

void FPVLiberator::publish_frame(const cv::Mat& frame) {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    image_pub.publish(msg);
}

void FPVLiberator::transfer_cb(struct libusb_transfer *xfer) {
    ctx* c = (ctx*) xfer->user_data;
    
    if (xfer->status != LIBUSB_TRANSFER_COMPLETED) {
        cerr << "Transfer failed: " << libusb_error_name(xfer->status) << endl;
        return;
    }

    c->last_packet = chrono::steady_clock::now();

    FPVLiberator* liberator = static_cast<FPVLiberator*>(c->decoder_ctx->c->opaque);
    liberator->decode(c->decoder_ctx, xfer->buffer, xfer->actual_length);

    if (c->shm_offset + xfer->actual_length > SHM_SIZE) {
        c->shm_offset = 0;
    }
    memcpy((char*)c->shm_ptr + c->shm_offset, xfer->buffer, xfer->actual_length);
    c->shm_offset += xfer->actual_length;

    if ((liberator->now() - c->last) > (c->bytes? 0.1 : 2)) {
        cerr << "rx " << (c->bytes / (liberator->now() - c->last))/1000 << " kb/s" << endl;
        c->last = liberator->now();
        c->bytes = 0;
    }
    c->bytes += xfer->actual_length;

    if (libusb_submit_transfer(xfer) < 0) {
        cerr << "Failed to resubmit transfer" << endl;
    }
}

int FPVLiberator::run() {
    while(ros::ok()) {
        if (!dh) {
            cerr << "Opening device" << endl;
            dh = open_device();
            if (!dh) {
                cerr << "Failed to open device, retrying in 5 seconds..." << endl;
                this_thread::sleep_for(chrono::seconds(5));
                continue;
            }

            if (xfr) libusb_free_transfer(xfr);
            xfr = libusb_alloc_transfer(0);
            libusb_fill_bulk_transfer(xfr, dh, 0x84, buf.data(), buf.size(), transfer_cb, &myctx, 1000);
            
            if (libusb_submit_transfer(xfr) < 0) {
                cerr << "Failed to submit initial transfer" << endl;
                close_device(dh);
                dh = NULL;
                continue;
            }
        }
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        int completed = 0;
        if (libusb_handle_events_timeout_completed(NULL, &tv, &completed) != LIBUSB_SUCCESS) {
            cerr << "Error in event handling" << endl;
            close_device(dh);
            dh = NULL;
            continue;
        }

        auto now = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::seconds>(now - myctx.last_packet).count() > 5) {
            cerr << "No packets received for 5 seconds, reconnecting..." << endl;
            close_device(dh);
            dh = NULL;
            continue;
        }

        ros::spinOnce();

        if (waitKey(1) == 27) break;  // Exit if ESC is pressed
    }
    return 0;
}

int main(int argc, char** argv) {
    FPVLiberator liberator(argc, argv);
    return liberator.run();
}