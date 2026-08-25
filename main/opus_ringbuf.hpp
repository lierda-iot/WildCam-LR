#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include "esp_heap_caps.h"
#include "app_config.h"

struct OpusFrameEntry {
    uint8_t len;
    uint8_t data[APP_OPUS_MAX_PACKET_BYTES];
};

class OpusRingBuf {
public:
    bool init(size_t max_frames)
    {
        buf_ = static_cast<OpusFrameEntry *>(
            heap_caps_malloc(max_frames * sizeof(OpusFrameEntry), MALLOC_CAP_SPIRAM));
        if (!buf_) return false;
        capacity_ = max_frames;
        std::memset(buf_, 0, max_frames * sizeof(OpusFrameEntry));
        return true;
    }

    void write(const uint8_t *data, uint8_t len)
    {
        buf_[write_pos_].len = len;
        std::memcpy(buf_[write_pos_].data, data, len);
        write_pos_++;
        if (write_pos_ >= capacity_) write_pos_ = 0;
        filled_++;
        if (filled_ > capacity_) filled_ = capacity_;
    }

    size_t snapshot(uint8_t *out, size_t max_bytes) const
    {
        size_t avail = (filled_ < capacity_) ? filled_ : capacity_;
        size_t start = (write_pos_ + capacity_ - avail) % capacity_;
        size_t total = 0;
        for (size_t i = 0; i < avail; i++) {
            const OpusFrameEntry &e = buf_[(start + i) % capacity_];
            if (e.len == 0) continue;
            if (total + 1 + e.len > max_bytes) break;
            out[total++] = e.len;
            std::memcpy(&out[total], e.data, e.len);
            total += e.len;
        }
        return total;
    }

private:
    OpusFrameEntry *buf_ = nullptr;
    size_t capacity_ = 0;
    size_t write_pos_ = 0;
    size_t filled_ = 0;
};
