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

using namespace std;

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

    // Open a pipe to ffplay for video decoding and display
    // "ffplay -fflags nobuffer -flags low_delay -i pipe:0 -analyzeduration 0 -probesize 32 -sync ext" for normal playback
    // "ffplay -fflags nobuffer -flags low_delay -framedrop -i pipe:0 -analyzeduration 0 -probesize 32 -sync ext -vf setpts=0" for faster playback
    // "ffplay -fflags nobuffer -flags low_delay -framedrop -i pipe:0 -analyzeduration 0 -probesize 32 -sync ext -vf setpts=0 -x 1280 -y 720" for hardware decoding playback
    int ffplay_pipe = fileno(popen("ffplay -fflags nobuffer -flags low_delay -framedrop -i pipe:0 -analyzeduration 0 -probesize 32 -sync ext -vf setpts=0", "w")); 
    if (ffplay_pipe == -1) {
        return die("Failed to open pipe to ffplay ", 1);
    }

    // Set the pipe to non-blocking mode
    int flags = fcntl(ffplay_pipe, F_GETFL, 0);
    fcntl(ffplay_pipe, F_SETFL, flags | O_NONBLOCK);

    struct ctx {
        double last;
        int bytes;
        int pipe;
    };
    ctx myctx = {now(), 0, ffplay_pipe};

    struct libusb_transfer *xfr = libusb_alloc_transfer(0);
    vector<uint8_t> buf(65536);  // Increased buffer size buffer size from 16384 to 32768 or 65536 bytes.
    libusb_fill_bulk_transfer(xfr, dh, 0x84, buf.data(), buf.size(), [](struct libusb_transfer *xfer){
        ctx* c = (ctx*) xfer->user_data;
        
        // Write received data to ffplay pipe
        int written = write(c->pipe, xfer->buffer, xfer->actual_length);
        if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            cerr << "Error writing to pipe: " << strerror(errno) << endl;
        }
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
    }
    cerr << "closing" << endl;
    libusb_exit(NULL);
    close(ffplay_pipe);
    return 0;
}