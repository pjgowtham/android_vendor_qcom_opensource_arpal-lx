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

#define LOG_TAG "PAL: Tfa98xx"
#include "Tfa98xx.h"
#include <log/log.h>
#include <unistd.h>
#include "Stream.h"
#include "Session.h"
#include "PalCommon.h"

Tfa98xx::Tfa98xx() {
    initialize();
}

bool Tfa98xx::initialize() {
    std::lock_guard<std::mutex> lock(mutex_);
    ALOGI("Initializing TFA98xx");

    if (initialized_) {
        ALOGD("Already initialized");
        return true;
    }

    if (!initializeMixers()) {
        ALOGE("Failed to initialize mixers");
        return false;
    }

    initializeCalibration();
    initialized_ = true;
    ALOGD("Initialization complete");
    return true;
}

bool Tfa98xx::initializeMixers() {
    auto resource_manager = ResourceManager::getInstance();
    if (!resource_manager) {
        ALOGE("Failed to get ResourceManager instance");
        return false;
    }

    struct audio_mixer* hw_mixer = nullptr;
    int status = resource_manager->getHwAudioMixer(&hw_mixer);
    if (status || !hw_mixer) {
        ALOGE("Failed to get hardware mixer");
        return false;
    }
    hw_mixer_ = hw_mixer;

    struct audio_mixer* virtual_mixer = nullptr;
    status = resource_manager->getVirtualAudioMixer(&virtual_mixer);
    if (status || !virtual_mixer) {
        ALOGE("Failed to get virtual mixer");
        return false;
    }
    virtual_mixer_ = virtual_mixer;

    return true;
}

void Tfa98xx::initializeCalibration() {
    struct mixer_ctl* calibration_control = nullptr;
    struct mixer_ctl* impedance_control = nullptr;

    if (hw_mixer_) {
        calibration_control = mixer_get_ctl_by_name(hw_mixer_, "TFA Calibration");
        if (!calibration_control) {
            ALOGE("Invalid mixer control: TFA Calibration");
            return;
        }

        speaker_count_ = mixer_ctl_get_num_values(calibration_control);
        if (speaker_count_ <= 0) {
            ALOGE("Invalid speaker count: %d", speaker_count_);
            return;
        }

        power_amp_count_ = std::min((speaker_count_ + 1) / 2, static_cast<int>(MAX_PA_COUNT));
        impedance_control = mixer_get_ctl_by_name(hw_mixer_, "TFA Default Impedance");
    }

    // Read calibration info for each speaker
    for (int i = 0; i < speaker_count_; i++) {
        FILE* fp = openDeviceFile(DEVICE_ADDRESSES[i], "cali_info");
        if (!fp) continue;

        char buffer[64] = {0};
        if (readDeviceFile(buffer, sizeof(buffer), fp) > 0) {
            CalibrationInfo info = {};
            int device_idx, min_ohms, max_ohms;

            if (sscanf(buffer, "0x%*x, %d, %d, %d\n", &device_idx, &min_ohms, &max_ohms) == 3) {
                info.device_index = device_idx;
                info.min_ohms = min_ohms;
                info.max_ohms = max_ohms;
                info.address = DEVICE_ADDRESSES[i];
                calibration_info_.push_back(info);
            }
        }
        fclose(fp);
    }

    // Sort and validate calibration info
    std::sort(calibration_info_.begin(), calibration_info_.end(), compareCalibrationInfo);

    if (calibration_info_.size() != speaker_count_) {
        ALOGE("Calibration info count (%zu) doesn't match speaker count (%d)",
              calibration_info_.size(), speaker_count_);
    }

    // Update impedance values
    for (auto& info : calibration_info_) {
        if (impedance_control) {
            int default_ohms = mixer_ctl_get_value(impedance_control, info.device_index);
            if (default_ohms > info.min_ohms && default_ohms < info.max_ohms) {
                info.default_impedance = default_ohms;
            }
        }

        ALOGD("Speaker %d: addr=0x%x, min=%d, max=%d, def=%d", 
              info.device_index, info.address, info.min_ohms, 
              info.max_ohms, info.default_impedance);
    }
}

FILE* Tfa98xx::openDeviceFile(uint8_t address, const char* type) {
    char path[128] = {0};
    snprintf(path, sizeof(path), "/proc/tfa98xx-%x/%s", address, type);

    FILE* fp = fopen(path, "r");
    if (!fp) {
        ALOGD("Failed to open %s for device 0x%x", type, address);
        return nullptr;
    }
    return fp;
}

long Tfa98xx::readDeviceFile(char* buffer, size_t size, FILE* fp) {
    if (!buffer || !fp || size == 0) {
        ALOGE("Invalid file read parameters");
        return 0;
    }

    size_t bytes = fread(buffer, 1, size - 1, fp);
    if (bytes == 0) {
        ALOGE("Failed to read file");
        return 0;
    }

    buffer[bytes] = '\0';
    return bytes;
}

int Tfa98xx::readDspMessage(uint8_t* buffer, int buffer_size) {
    if (!buffer || buffer_size <= 0) {
        ALOGE("Invalid read parameters: buffer=%p, size=%d", buffer, buffer_size);
        return -EINVAL;
    }

    mixer_ctl* mixer_control = nullptr;
    if (hw_mixer_) {
        mixer_control = mixer_get_ctl_by_name(hw_mixer_, "getParam");
    }
    if (!mixer_control) {
        ALOGE("Failed to get mixer control for read operation");
        return -EINVAL;
    }

    size_t total_size = DspMessage::calculateSize(buffer_size);
    std::unique_ptr<uint8_t[]> message_buffer(new (std::nothrow) uint8_t[total_size]());
    if (!message_buffer) {
        ALOGE("Failed to allocate message buffer");
        return -ENOMEM;
    }

    DspMessage* dsp_message = reinterpret_cast<DspMessage*>(message_buffer.get());
    dsp_message->module_instance_id = TFA98XX_MI_ID_TAG;
    dsp_message->header = TFA98XX_DSP_MSG_READ_HEADER;
    dsp_message->payload_size = total_size - DspMessage::headerSize();
    dsp_message->reserved = 0;

    int status = mixer_ctl_get_array(mixer_control, dsp_message, total_size);
    if (status != 0) {
        ALOGE("Failed to read DSP response: %d", status);
        return status;
    }

    memcpy(buffer, dsp_message->payload, buffer_size);
    return 0;
}

int Tfa98xx::writeDspMessage(const uint8_t* message, int message_size) {
    if (!message && message_size > 0) {
        ALOGE("Invalid write parameters: message=%p, size=%d", message, message_size);
        return -EINVAL;
    }

    mixer_ctl* mixer_control = nullptr;
    if (hw_mixer_) {
        mixer_control = mixer_get_ctl_by_name(hw_mixer_, "setParam");
    }
    if (!mixer_control) {
        ALOGE("Failed to get mixer control for write operation");
        return -EINVAL;
    }

    size_t total_size = DspMessage::calculateSize(message_size);
    std::unique_ptr<uint8_t[]> message_buffer(new (std::nothrow) uint8_t[total_size]());
    if (!message_buffer) {
        ALOGE("Failed to allocate message buffer");
        return -ENOMEM;
    }

    DspMessage* dsp_message = reinterpret_cast<DspMessage*>(message_buffer.get());
    dsp_message->module_instance_id = TFA98XX_MI_ID_TAG;
    dsp_message->header = TFA98XX_DSP_MSG_WRITE_HEADER;
    dsp_message->payload_size = total_size - DspMessage::headerSize();
    dsp_message->reserved = 0;

    if (message && message_size > 0) {
        memcpy(dsp_message->payload, message, message_size);
    }

    int status = mixer_ctl_set_array(mixer_control, dsp_message, total_size);
    if (status != 0) {
        ALOGE("Failed to write DSP message: %d", status);
        return status;
    }

    usleep(TFA98XX_MSG_WRITE_DELAY_US);
    return 0;
}

int Tfa98xx::getR0(uint32_t& r0_left, uint32_t& r0_right) {
    std::lock_guard<std::mutex> lock(mutex_);
    ALOGD("%s: entry point", __func__);
    
    if (!initialized_) {
        ALOGE("%s: TFA98xx not initialized", __func__);
        return -EINVAL;
    }
    
    // Send empty message to prepare DSP
    int status = writeDspMessage(nullptr, DspR0Message::MESSAGE_SIZE + 3); // 0x18
    if (status < 0) {
        ALOGE("%s: send memtrack failed", __func__);
        return status;
    }
    
    // Wait for DSP to process
    usleep(DSP_RESPONSE_DELAY_US);  // Using constant for 50000us
    
    // Prepare and send R0 measurement command
    DspR0Message msg = {};
    msg.prepareGetR0();
    
    status = writeDspMessage(reinterpret_cast<const uint8_t*>(&msg), 
                           DspR0Message::MESSAGE_SIZE);
    if (status < 0) {
        ALOGE("%s: send memtrack failed", __func__);
        return status;
    }
    
    // Read response
    status = readDspMessage(reinterpret_cast<uint8_t*>(&msg), 
                          DspR0Message::MESSAGE_SIZE);
    if (status < 0) {
        ALOGE("%s: receive command failed", __func__);
        return status;
    }
    
    // Parse response using the exact same bit manipulation as original
    auto [left, right] = msg.parseR0Response();
    r0_left = left;
    r0_right = right;
    
    ALOGD("%s: R0 values - Left: %u, Right: %u", __func__, left, right);
    return 0;
}

int Tfa98xx::getF0(uint32_t& f0_left, uint32_t& f0_right) {
    std::lock_guard<std::mutex> lock(mutex_);
    ALOGD("%s: entry point", __func__);
    
    if (!initialized_) {
        ALOGE("%s: TFA98xx not initialized", __func__);
        return -EINVAL;
    }
    
    // Send empty message to prepare DSP
    int status = writeDspMessage(nullptr, DspF0Message::MESSAGE_SIZE + 3); // 0x18
    if (status < 0) {
        ALOGE("%s: send memtrack failed", __func__);
        return status;
    }
    
    // Wait for DSP to process
    usleep(DSP_RESPONSE_DELAY_US);  // Using constant for 50000us
    
    // Prepare and send F0 measurement command
    DspF0Message msg = {};
    msg.prepareGetF0();
    
    status = writeDspMessage(reinterpret_cast<const uint8_t*>(&msg), 
                           DspF0Message::MESSAGE_SIZE);
    if (status < 0) {
        ALOGE("%s: send memtrack failed", __func__);
        return status;
    }
    
    // Read response
    status = readDspMessage(reinterpret_cast<uint8_t*>(&msg), 
                          DspF0Message::MESSAGE_SIZE);
    if (status < 0) {
        ALOGE("%s: receive command failed", __func__);
        return status;
    }
    
    // Parse response using the exact same bit manipulation as original
    auto [left, right] = msg.parseF0Response();
    f0_left = left;
    f0_right = right;
    
    ALOGD("%s: F0 values - Left: %u, Right: %u", __func__, left, right);
    return 0;
}

void Tfa98xx::setVolume(uint8_t volume) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        ALOGE("%s: TFA98xx not initialized", __func__);
        return;
    }
    
    DspVolumeMessage msg = {};
    msg.prepareSetVolume(volume);
    
    int status = writeDspMessage(reinterpret_cast<const uint8_t*>(&msg),
                               DspVolumeMessage::MESSAGE_SIZE);
    if (status < 0) {
        ALOGE("%s: Failed to set volume", __func__);
    }
}

int Tfa98xx::getMiid(const char* param_name, struct mixer_ctl** mixer_control, uint32_t* miid) {
    if (!param_name || !mixer_control || !miid) {
        ALOGE("%s: Invalid parameters", __func__);
        return -EINVAL;
    }

    auto rm = ResourceManager::getInstance();
    if (!rm) {
        ALOGE("%s: Failed to get resource manager instance", __func__);
        return -EINVAL;
    }

    // Get active stream
    std::vector<Stream*> activeStreams;
    std::shared_ptr<Device> dev = nullptr;
    int status = rm->getActiveStream_l(activeStreams, dev);
    if (status != 0 || activeStreams.empty()) {
        ALOGE("%s: no active stream available", __func__);
        return -EINVAL;
    }

    // Get session from stream
    Session* session = nullptr;
    Stream* stream = static_cast<Stream*>(activeStreams[0]);
    stream->getAssociatedSession(&session);
    if (!session) {
        ALOGE("%s: Failed to get session", __func__);
        return -EINVAL;
    }

    // For voice streams, construct special mixer control name
    std::string mixer_name;
    pal_stream_type_t stream_type;
    if (stream->getStreamType(&stream_type) == 0 && 
        stream_type == PAL_STREAM_VOICE_CALL) {
        std::string voice_prefix;
        uint32_t voice_mode = 0; // Get this from stream attributes
        if (voice_mode == 0x121c6000 || voice_mode == 0x11dc5000) {
            voice_prefix = "VOICEMMODE2p";
        } else {
            voice_prefix = "VOICEMMODE1p";
        }
        mixer_name = voice_prefix + " " + param_name;
    } else {
        mixer_name = param_name;
    }

    // Get mixer control
    struct audio_mixer* virtual_mixer = nullptr;
    rm->getVirtualAudioMixer(&virtual_mixer);
    if (!virtual_mixer) {
        ALOGE("%s: Failed to get virtual mixer", __func__);
        return -EINVAL;
    }

    *mixer_control = mixer_get_ctl_by_name(virtual_mixer, mixer_name.c_str());
    if (!*mixer_control) {
        ALOGE("%s: Invalid mixer control: %s", __func__, mixer_name.c_str());
        return -EINVAL;
    }

    // Get backend name and MIID
    std::string backend_name;
    status = getBackendName(param_name, backend_name);
    if (status != 0) {
        ALOGE("%s: Failed to get backend name", __func__);
        return status;
    }

    status = session->getMIID(backend_name.c_str(), MODULE_VI, miid);
    if (status != 0) {
        ALOGE("%s: Failed to get tag info %x, backend %s status = %d",
              __func__, TFA98XX_MI_ID_TAG, backend_name.c_str(), status);
    }

    return status;
}

int Tfa98xx::getBackendName(const char* param_name, std::string& backend_name) {
    auto rm = ResourceManager::getInstance();
    if (!rm) {
        ALOGE("%s: Failed to get resource manager instance", __func__);
        return -EINVAL;
    }

    uint32_t device_id = 0; // Get this from your device attributes
    if (device_id == 1 || (device_id > 0x14 && device_id < 0x16)) {
        ALOGE("%s: Invalid device id %d", __func__, device_id);
        return -EINVAL;
    }

    rm->getBackendName(device_id, backend_name);
    return 0;
}

