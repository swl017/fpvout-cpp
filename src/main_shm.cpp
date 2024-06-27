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

using namespace std;
using namespace cv;

#define SHM_NAME "/fpv_liberator_shm"
#define SHM_SIZE 1048576  // 1MB shared memory size
#define FRAME_WIDTH 640   // Adjust these values according to your video format
#define FRAME_HEIGHT 480

double now() {
    struct timeb timebuffer;
    ftime(&timebuffer);
    return timebuffer.time + ((double)timebuffer.millitm)/1000;
}

int die(string msg, int code) { cerr << msg << code << endl; return code; }

int main() {
    cerr << "Using libusb v" << libusb_get_version()->major << "." << libusb_get_version()->minor << "." << libusb_get_version()->micro << endl;
    int err = 0;
    cerr << "initializing" << endl;
    if ((err = libusb_init(NULL)) < 0)
        return die("initialize fail ", err);
    cerr << "opening device" << endl;
    libusb_device_handle *dh = libusb_open_device_with_vid_pid(NULL, 0x2ca3, 0x001f);
    if (!dh) return die("open device fail ", 1);

    cerr << "claiming interface" << endl;
    if ((err = libusb_claim_interface(dh, 3)) < 0)
        return die("claim interface fail ", err);

    cerr << "sending magic packet" << endl;
    int tx = 0;
    uint8_t data[] = {0x52, 0x4d, 0x56, 0x54};
    if ((err = libusb_bulk_transfer(dh, 0x03, data, 4, &tx, 500)) < 0)
        cerr << "ERROR: No data transmitted to device " << err << endl; //don't exit

    // Create shared memory
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

    // Start ffplay in a separate process
    pid_t pid = fork();
    if (pid == 0) {  // Child process
        // Close the write end of the shared memory in the child
        close(shm_fd);
        
        // Execute ffplay
        execlp("ffplay", "ffplay", "-fflags", "nobuffer", "-flags", "low_delay", "-framedrop",
               "-i", SHM_NAME, "-analyzeduration", "0", "-probesize", "32", "-sync", "ext", NULL);
        
        // If execlp returns, it means it failed
        exit(1);
    } else if (pid < 0) {
        return die("Failed to fork ", errno);
    }

    // Create OpenCV window for debug output
    namedWindow("Debug Output", WINDOW_NORMAL);

    struct ctx {
        double last;
        int bytes;
        void* shm_ptr;
        size_t shm_offset;
        Mat frame;
    };
    ctx myctx = {now(), 0, shm_ptr, 0, Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3)};

    struct libusb_transfer *xfr = libusb_alloc_transfer(0);
    vector<uint8_t> buf(65536);  // Increased buffer size as per previous advice
    libusb_fill_bulk_transfer(xfr, dh, 0x84, buf.data(), buf.size(), [](struct libusb_transfer *xfer){
        ctx* c = (ctx*) xfer->user_data;
        
        // Copy received data to shared memory
        if (c->shm_offset + xfer->actual_length > SHM_SIZE) {
            c->shm_offset = 0;  // Wrap around if we reach the end of the shared memory
        }
        memcpy((char*)c->shm_ptr + c->shm_offset, xfer->buffer, xfer->actual_length);
        c->shm_offset += xfer->actual_length;

        // Update OpenCV frame
        memcpy(c->frame.data, xfer->buffer, min(xfer->actual_length, (int)(FRAME_WIDTH * FRAME_HEIGHT * 3)));

        // Display the frame
        imshow("Debug Output", c->frame);
        waitKey(1);  // This allows the GUI to update

        if ((now() - c->last) > (c->bytes? 0.1 : 2)) {
            cerr << "rx " << (c->bytes / (now() - c->last))/1000 << " kb/s" << endl;
            c->last = now();
            c->bytes = 0;
        }
        c->bytes += xfer->actual_length;
        int err;
        if ((err = libusb_submit_transfer(xfer)) < 0) //do another read
            exit(die("read init fail ", err));
    }, &myctx, 100);
    if ((err = libusb_submit_transfer(xfr)) < 0)
        return die("read init fail ", err);

    while(1) {
        if (libusb_handle_events(NULL) != LIBUSB_SUCCESS) break;
        if (waitKey(1) == 27) break;  // Exit if ESC is pressed
    }
    cerr << "closing" << endl;
    libusb_exit(NULL);

    // Clean up shared memory
    munmap(shm_ptr, SHM_SIZE);
    close(shm_fd);
    shm_unlink(SHM_NAME);

    // Clean up OpenCV
    destroyAllWindows();

    return 0;
}