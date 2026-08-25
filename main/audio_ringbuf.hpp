#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include "esp_heap_caps.h"

class AudioRingBuf {
public:
    bool init(size_t max_samples)
    {
        buf_ = static_cast<int16_t *>(
            heap_caps_malloc(max_samples * sizeof(int16_t), MALLOC_CAP_SPIRAM));
        if (!buf_) return false;
        capacity_ = max_samples;
        std::memset(buf_, 0, max_samples * sizeof(int16_t));
        return true;
    }

    void write(const int16_t *samples, size_t count)
    {
        for (size_t i = 0; i < count; i++) {
            buf_[write_pos_] = samples[i];
            write_pos_++;
            if (write_pos_ >= capacity_) write_pos_ = 0;
        }
        filled_ += count;
        if (filled_ > capacity_) filled_ = capacity_;
    }

    size_t snapshot(int16_t *out, size_t max_samples) const
    {
        size_t avail = (filled_ < max_samples) ? filled_ : max_samples;
        size_t start = (write_pos_ + capacity_ - avail) % capacity_;
        for (size_t i = 0; i < avail; i++) {
            out[i] = buf_[(start + i) % capacity_];
        }
        return avail;
    }

private:
    int16_t *buf_ = nullptr;
    size_t capacity_ = 0;
    size_t write_pos_ = 0;
    size_t filled_ = 0;
};
