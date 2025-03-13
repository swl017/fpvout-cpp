#include "fpvout.h"

FPVout::FPVout(int width, int height) : dh(NULL), xfr(NULL), buf(65536), width_(width), height_(height) {
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
        Mat(height_, width_, CV_8UC3),
        chrono::steady_clock::now(),
        decoder_ctx
    };
}

FPVout::~FPVout() {
    cerr << "closing" << endl;
    if (dh) close_device(dh);
    libusb_exit(NULL);

    munmap(myctx.shm_ptr, SHM_SIZE);
    close(shm_fd);
    shm_unlink(SHM_NAME);

    destroyAllWindows();
    close_decoder(myctx.decoder_ctx);
}

FPVout::DecoderContext* FPVout::init_decoder() {
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

void FPVout::decode(DecoderContext* ctx, uint8_t* data, int data_size) {
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
            }
        }
    }
}

void FPVout::close_decoder(DecoderContext* ctx) {
    av_parser_close(ctx->parser);
    avcodec_free_context(&ctx->c);
    av_frame_free(&ctx->frame);
    av_packet_free(&ctx->pkt);
    sws_freeContext(ctx->sws_ctx);
    delete ctx;
}

double FPVout::now() {
    struct timeb timebuffer;
    ftime(&timebuffer);
    return timebuffer.time + ((double)timebuffer.millitm)/1000;
}

int FPVout::die(string msg, int code) { 
    cerr << msg << code << endl; 
    return code; 
}

libusb_device_handle* FPVout::open_device() {
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

void FPVout::close_device(libusb_device_handle* dh) {
    if (dh) {
        libusb_release_interface(dh, 3);
        libusb_close(dh);
    }
}

void FPVout::transfer_cb(struct libusb_transfer *xfer) {
    ctx* c = (ctx*) xfer->user_data;
    
    if (xfer->status != LIBUSB_TRANSFER_COMPLETED) {
        cerr << "Transfer failed: " << libusb_error_name(xfer->status) << endl;
        return;
    }

    c->last_packet = chrono::steady_clock::now();

    FPVout* liberator = static_cast<FPVout*>(c->decoder_ctx->c->opaque);
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

int FPVout::run() {
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

        auto now = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::seconds>(now - myctx.last_packet).count() > 5) {
            cerr << "No packets received for 5 seconds, reconnecting..." << endl;
            close_device(dh);
            dh = NULL;
            continue;
        }

        if (waitKey(1) == 27) break;  // Exit if ESC is pressed
    }
    return 0;
}