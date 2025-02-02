/*
 * Copyright (C) 2025 The LineageOS Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <algorithm>
#include <cstdio>
#include "ResourceManager.h"
#include "Stream.h"
#include "Session.h"

class Tfa98xx {
public:
    Tfa98xx();
    ~Tfa98xx() = default;
    Tfa98xx(const Tfa98xx&) = delete;
    Tfa98xx& operator=(const Tfa98xx&) = delete;

    bool initialize();
    int getR0(uint32_t& r0_left, uint32_t& r0_right);
    int getF0(uint32_t& f0_left, uint32_t& f0_right);
    int readDspMessage(uint8_t* buffer, int buffer_size);
    int writeDspMessage(const uint8_t* message, int message_size);
    void setVolume(uint8_t volume);

private:
    static constexpr uint8_t DEVICE_ADDRESSES[] = {0x34, 0x35, 0x36, 0x37};
    static constexpr uint32_t TFA98XX_MI_ID_TAG = 0xC0000029;
    static constexpr uint32_t TFA98XX_DSP_MSG_WRITE_HEADER = 0x1800b921;
    static constexpr uint32_t TFA98XX_DSP_MSG_READ_HEADER = 0x1800b922;
    static constexpr uint32_t TFA98XX_MSG_WRITE_DELAY_US = 5000;
    static constexpr uint32_t DSP_RESPONSE_DELAY_US = 50000;
    static constexpr size_t MAX_PA_COUNT = 2;
    static constexpr float R0_TOLERANCE_PERCENT = 15.0f;
    static constexpr uint32_t MIN_F0_HZ = 400;
    static constexpr uint32_t MAX_F0_HZ = 1000;

    struct CalibrationInfo {
        int device_index;
        int calibrated_impedance;
        int default_impedance;
        int dsp_impedance;
        uint8_t address;
        int min_ohms;
        int max_ohms;
    };

    struct DspMessage {
        uint32_t module_instance_id;
        uint32_t header;
        uint32_t payload_size;
        uint32_t reserved;
        uint8_t payload[];

        static size_t calculateSize(int message_size) {
            return (sizeof(DspMessage) + message_size + 7) & ~7;
        }

        static constexpr size_t headerSize() {
            return sizeof(DspMessage);
        }
    };

    struct DspR0Message {
        uint64_t header;
        uint8_t padding[7];
        uint8_t data[9];
        
        static constexpr uint32_t GET_R0_CMD = 0x8b8000;
        static constexpr size_t MESSAGE_SIZE = 0x15;
        
        void prepareGetR0() {
            memset(this, 0, sizeof(*this));
            header = GET_R0_CMD;
        }
        
        std::pair<uint32_t, uint32_t> parseR0Response() const {
            uint32_t r0_left = (data[0] << 16) | (data[1] << 8) | data[2];
            uint32_t r0_right = (data[3] << 16) | (data[4] << 8) | data[5];
            return {r0_left, r0_right};
        }
    };

    struct DspF0Message {
        uint64_t header;
        uint8_t padding[7];
        uint8_t data[9];
        
        static constexpr uint32_t GET_F0_CMD = 0x8b8000;
        static constexpr size_t MESSAGE_SIZE = 0x15;
        
        void prepareGetF0() {
            memset(this, 0, sizeof(*this));
            header = GET_F0_CMD;
        }
        
        std::pair<uint32_t, uint32_t> parseF0Response() const {
            uint32_t f0_left = (data[0] << 16) | (data[1] << 8) | data[2];
            uint32_t f0_right = (data[3] << 16) | (data[4] << 8) | data[5];
            return {f0_left, f0_right};
        }
    };

    struct DspVolumeMessage {
        uint64_t header;     // To store CONCAT15(volume, 0x48100)
        uint8_t volume;      // To store the volume value
        
        static constexpr uint32_t SET_VOLUME_CMD = 0x48100;
        static constexpr size_t MESSAGE_SIZE = 9;  // Size from original
        
        void prepareSetVolume(uint8_t vol) {
            header = SET_VOLUME_CMD;
            volume = vol;
        }
    };

    bool initializeMixers();
    void initializeCalibration();
    FILE* openDeviceFile(uint8_t address, const char* type);
    long readDeviceFile(char* buffer, size_t size, FILE* fp);

    static bool compareCalibrationInfo(const CalibrationInfo& a, const CalibrationInfo& b) {
        return a.device_index < b.device_index;
    }

    std::mutex mutex_;
    bool initialized_{false};
    struct audio_mixer* hw_mixer_{nullptr};
    struct audio_mixer* virtual_mixer_{nullptr};
    std::vector<CalibrationInfo> calibration_info_;
    int power_amp_count_{0};
    int speaker_count_{0};

    int getMiid(const char* param_name, struct mixer_ctl** mixer_control, uint32_t* miid);
    int getBackendName(const char* param_name, std::string& backend_name);
};
