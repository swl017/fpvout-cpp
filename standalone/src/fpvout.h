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

#define SHM_NAME "/fpvout_shm"
#define SHM_SIZE 1048576  // 1MB shared memory size
#define FRAME_WIDTH 1280   // Adjust these values according to your video format
#define FRAME_HEIGHT 720
#define MAX_RETRIES 5
#define INBUF_SIZE 4096

class FPVout {
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

    DecoderContext* init_decoder();
    void decode(DecoderContext* ctx, uint8_t* data, int data_size);
    void close_decoder(DecoderContext* ctx);
    double now();
    int die(string msg, int code);
    libusb_device_handle* open_device();
    void close_device(libusb_device_handle* dh);
    static void transfer_cb(struct libusb_transfer *xfer);

public:
    FPVout(int width, int height);
    ~FPVout();
    int run();
    int width_;
    int height_;
};
