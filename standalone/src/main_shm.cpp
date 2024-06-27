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
#define INBUF_SIZE 4096

using namespace std;
using namespace cv;

#define SHM_NAME "/fpv_liberator_shm"
#define SHM_SIZE 1048576  // 1MB shared memory size
#define FRAME_WIDTH 640   // Adjust these values according to your video format
#define FRAME_HEIGHT 480
#define MAX_RETRIES 5
struct DecoderContext {
    const AVCodec *codec;
    AVCodecParserContext *parser;
    AVCodecContext *c;
    AVFrame *frame;
    AVPacket *pkt;
    SwsContext *sws_ctx;
};

DecoderContext* init_decoder() {
    DecoderContext* ctx = new DecoderContext();

    ctx->codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!ctx->codec) {
        std::cerr << "Codec not found" << std::endl;
        return nullptr;
    }

    ctx->parser = av_parser_init(ctx->codec->id);
    if (!ctx->parser) {
        std::cerr << "Parser not found" << std::endl;
        return nullptr;
    }

    ctx->c = avcodec_alloc_context3(ctx->codec);
    if (!ctx->c) {
        std::cerr << "Could not allocate video codec context" << std::endl;
        return nullptr;
    }

    if (avcodec_open2(ctx->c, ctx->codec, NULL) < 0) {
        std::cerr << "Could not open codec" << std::endl;
        return nullptr;
    }

    ctx->frame = av_frame_alloc();
    ctx->pkt = av_packet_alloc();

    return ctx;
}

void decode(DecoderContext* ctx, uint8_t* data, int data_size) {
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

                // Convert the frame to BGR format for OpenCV
                if (!ctx->sws_ctx) {
                    ctx->sws_ctx = sws_getContext(ctx->c->width, ctx->c->height, ctx->c->pix_fmt,
                                                  ctx->c->width, ctx->c->height, AV_PIX_FMT_BGR24,
                                                  SWS_BILINEAR, NULL, NULL, NULL);
                }

                cv::Mat frame(ctx->c->height, ctx->c->width, CV_8UC3);
                uint8_t *dest[4] = {frame.data, NULL, NULL, NULL};
                int dest_linesize[4] = {frame.step, 0, 0, 0};
                sws_scale(ctx->sws_ctx, ctx->frame->data, ctx->frame->linesize, 0, ctx->c->height, dest, dest_linesize);

                // Display the frame
                cv::imshow("Decoded Frame", frame);
                cv::waitKey(1);
            }
        }
    }
}

void close_decoder(DecoderContext* ctx) {
    av_parser_close(ctx->parser);
    avcodec_free_context(&ctx->c);
    av_frame_free(&ctx->frame);
    av_packet_free(&ctx->pkt);
    sws_freeContext(ctx->sws_ctx);
    delete ctx;
}

double now() {
    struct timeb timebuffer;
    ftime(&timebuffer);
    return timebuffer.time + ((double)timebuffer.millitm)/1000;
}

int die(string msg, int code) { cerr << msg << code << endl; return code; }

libusb_device_handle* open_device() {
    int err = 0;
    libusb_device_handle *dh = libusb_open_device_with_vid_pid(NULL, 0x2ca3, 0x001f);
    if (dh) {
        if ((err = libusb_claim_interface(dh, 3)) < 0) {
            libusb_close(dh);
            die("claim interface fail ", err);
        }
        
        int tx = 0;
        uint8_t data[] = {0x52, 0x4d, 0x56, 0x54};
        if (libusb_bulk_transfer(dh, 0x03, data, 4, &tx, 500) < 0) {
            cerr << "Warning: No data transmitted to device" << endl;
        }
    }
    return dh;
}

void close_device(libusb_device_handle* dh) {
    if (dh) {
        libusb_release_interface(dh, 3);
        libusb_close(dh);
    }
}

int main() {
    cerr << "Using libusb v" << libusb_get_version()->major << "." << libusb_get_version()->minor << "." << libusb_get_version()->micro << endl;
    int err = 0;
    cerr << "initializing" << endl;
    if ((err = libusb_init(NULL)) < 0)
        return die("initialize fail ", err);

    // Create shared memory and set up ffplay (same as before)
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        return die("Failed to create shared memory ", errno);
    }

    // Configure the size of the shared memory object
    if (ftruncate(shm_fd, SHM_SIZE) == -1) {
        return die("Failed to configure shared memory size ", errno);
    }

    // Map the shared memory object into this process's memory space
    void* shm_ptr = mmap(0, SHM_SIZE, PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED) {
        return die("Failed to map shared memory ", errno);
    }

    // Create OpenCV window for debug output
    namedWindow("Debug Output", WINDOW_NORMAL);

    struct ctx {
        double last;
        int bytes;
        void* shm_ptr;
        size_t shm_offset;
        Mat frame;
        chrono::steady_clock::time_point last_packet;
        DecoderContext* decoder_ctx;  
    };
    DecoderContext* decoder_ctx = init_decoder();
    if (!decoder_ctx) {
        return -1;
    }
    ctx myctx = {
        now(),
        0,
        shm_ptr,
        0,
        Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3),
        chrono::steady_clock::now(),
        decoder_ctx
    };

    libusb_device_handle* dh = NULL;
    struct libusb_transfer *xfr = NULL;
    vector<uint8_t> buf(65536);

    auto transfer_cb = [](struct libusb_transfer *xfer) {
        ctx* c = (ctx*) xfer->user_data;
        
        if (xfer->status != LIBUSB_TRANSFER_COMPLETED) {
            cerr << "Transfer failed: " << libusb_error_name(xfer->status) << endl;
            return;
        }

        c->last_packet = chrono::steady_clock::now();

        // Decode the received data
        decode(c->decoder_ctx, xfer->buffer, xfer->actual_length);

        // Copy received data to shared memory
        if (c->shm_offset + xfer->actual_length > SHM_SIZE) {
            c->shm_offset = 0;  // Wrap around if we reach the end of the shared memory
        }
        memcpy((char*)c->shm_ptr + c->shm_offset, xfer->buffer, xfer->actual_length);
        c->shm_offset += xfer->actual_length;

        // // Assume the data is in YUV422 format (common for video devices)
        // cv::Mat yuv(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC2, xfer->buffer);
        // cv::Mat bgr;
        // cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_YUYV);

        // // Output the decoded frame
        // cv::imshow("Debug Output", bgr);

        // cv::waitKey(1);  // This allows the GUI to update

        if ((now() - c->last) > (c->bytes? 0.1 : 2)) {
            cerr << "rx " << (c->bytes / (now() - c->last))/1000 << " kb/s" << endl;
            c->last = now();
            c->bytes = 0;
        }
        c->bytes += xfer->actual_length;

        // Resubmit the transfer
        if (libusb_submit_transfer(xfer) < 0) {
            cerr << "Failed to resubmit transfer" << endl;
        }
    };
    while(1) {
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

        // Check if we haven't received a packet in a while
        auto now = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::seconds>(now - myctx.last_packet).count() > 5) {
            cerr << "No packets received for 5 seconds, reconnecting..." << endl;
            close_device(dh);
            dh = NULL;
            continue;
        }

        if (waitKey(1) == 27) break;  // Exit if ESC is pressed
    }

    cerr << "closing" << endl;
    if (dh) close_device(dh);
    libusb_exit(NULL);

    // Clean up shared memory
    munmap(shm_ptr, SHM_SIZE);
    close(shm_fd);
    shm_unlink(SHM_NAME);

    // Clean up OpenCV
    destroyAllWindows();

    return 0;
}