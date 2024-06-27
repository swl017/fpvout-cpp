// fpv_liberator.cpp
#include "fpv_liberator.h"

FPVLiberator::FPVLiberator() 
    : dh(nullptr), xfr(nullptr), buf(65536), shm_fd(-1), running(false) {
    myctx = {now(), 0, nullptr, 0, cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3), std::chrono::steady_clock::now()};
}

FPVLiberator::~FPVLiberator() {
    stop();
    if (myctx.shm_ptr) {
        munmap(myctx.shm_ptr, SHM_SIZE);
        close(shm_fd);
        shm_unlink(SHM_NAME);
    }
    libusb_exit(nullptr);
}

bool FPVLiberator::initialize() {
    if (libusb_init(nullptr) < 0) {
        std::cerr << "Failed to initialize libusb" << std::endl;
        return false;
    }

    shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        std::cerr << "Failed to create shared memory" << std::endl;
        return false;
    }

    if (ftruncate(shm_fd, SHM_SIZE) == -1) {
        std::cerr << "Failed to configure shared memory size" << std::endl;
        return false;
    }

    myctx.shm_ptr = mmap(0, SHM_SIZE, PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (myctx.shm_ptr == MAP_FAILED) {
        std::cerr << "Failed to map shared memory" << std::endl;
        return false;
    }

    return true;
}

void FPVLiberator::run() {
    running = true;
    event_thread = std::thread(&FPVLiberator::handle_events, this);
}

void FPVLiberator::stop() {
    running = false;
    if (event_thread.joinable()) {
        event_thread.join();
    }
    close_device();
}

cv::Mat FPVLiberator::getLatestFrame() {
    return myctx.frame.clone();
}

double FPVLiberator::now() {
    struct timeb timebuffer;
    ftime(&timebuffer);
    return timebuffer.time + ((double)timebuffer.millitm)/1000;
}

void FPVLiberator::transfer_cb(struct libusb_transfer *xfer) {
    ctx* c = static_cast<ctx*>(xfer->user_data);
    
    if (xfer->status != LIBUSB_TRANSFER_COMPLETED) {
        std::cerr << "Transfer failed: " << libusb_error_name(xfer->status) << std::endl;
        return;
    }

    c->last_packet = std::chrono::steady_clock::now();

    if (c->shm_offset + xfer->actual_length > SHM_SIZE) {
        c->shm_offset = 0;
    }
    std::memcpy(static_cast<char*>(c->shm_ptr) + c->shm_offset, xfer->buffer, xfer->actual_length);
    c->shm_offset += xfer->actual_length;

    std::memcpy(c->frame.data, xfer->buffer, std::min(xfer->actual_length, (int)(FRAME_WIDTH * FRAME_HEIGHT * 3)));

    if ((now() - c->last) > (c->bytes? 0.1 : 2)) {
        std::cerr << "rx " << (c->bytes / (now() - c->last))/1000 << " kb/s" << std::endl;
        c->last = now();
        c->bytes = 0;
    }
    c->bytes += xfer->actual_length;

    if (libusb_submit_transfer(xfer) < 0) {
        std::cerr << "Failed to resubmit transfer" << std::endl;
    }
}

libusb_device_handle* FPVLiberator::open_device() {
    libusb_device_handle *dh = libusb_open_device_with_vid_pid(nullptr, 0x2ca3, 0x001f);
    if (dh) {
        if (libusb_claim_interface(dh, 3) < 0) {
            libusb_close(dh);
            return nullptr;
        }
        
        int tx = 0;
        uint8_t data[] = {0x52, 0x4d, 0x56, 0x54};
        if (libusb_bulk_transfer(dh, 0x03, data, 4, &tx, 500) < 0) {
            std::cerr << "Warning: No data transmitted to device" << std::endl;
        }
    }
    return dh;
}

void FPVLiberator::close_device() {
    if (dh) {
        libusb_release_interface(dh, 3);
        libusb_close(dh);
        dh = nullptr;
    }
}

void FPVLiberator::handle_events() {
    while (running) {
        if (!dh) {
            std::cerr << "Opening device" << std::endl;
            dh = open_device();
            if (!dh) {
                std::cerr << "Failed to open device, retrying in 5 seconds..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(5));
                continue;
            }

            if (xfr) libusb_free_transfer(xfr);
            xfr = libusb_alloc_transfer(0);
            libusb_fill_bulk_transfer(xfr, dh, 0x84, buf.data(), buf.size(), transfer_cb, &myctx, 1000);
            
            if (libusb_submit_transfer(xfr) < 0) {
                std::cerr << "Failed to submit initial transfer" << std::endl;
                close_device();
                continue;
            }
        }

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        int completed = 0;
        if (libusb_handle_events_timeout_completed(NULL, &tv, &completed) != LIBUSB_SUCCESS) {
            std::cerr << "Error in event handling" << std::endl;
            close_device();
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - myctx.last_packet).count() > 5) {
            std::cerr << "No packets received for 5 seconds, reconnecting..." << std::endl;
            close_device();
            continue;
        }
    }
}
