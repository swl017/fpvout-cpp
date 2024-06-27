// fpv_liberator.h
#ifndef FPV_LIBERATOR_H
#define FPV_LIBERATOR_H

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

class FPVLiberator {
public:
    FPVLiberator();
    ~FPVLiberator();

    bool initialize();
    void run();
    void stop();
    cv::Mat getLatestFrame();

private:
    static constexpr const char* SHM_NAME = "/fpv_liberator_shm";
    static constexpr size_t SHM_SIZE = 1048576;  // 1MB shared memory size
    static constexpr int FRAME_WIDTH = 640;
    static constexpr int FRAME_HEIGHT = 480;

    struct ctx {
        double last;
        int bytes;
        void* shm_ptr;
        size_t shm_offset;
        cv::Mat frame;
        std::chrono::steady_clock::time_point last_packet;
    };

    ctx myctx;
    libusb_device_handle* dh;
    struct libusb_transfer *xfr;
    std::vector<uint8_t> buf;
    int shm_fd;
    bool running;
    std::thread event_thread;

    static double now();
    static void transfer_cb(struct libusb_transfer *xfer);
    libusb_device_handle* open_device();
    void close_device();
    void handle_events();
};

#endif // FPV_LIBERATOR_H