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

#define LOG_TAG "PAL: SpeakerProtectionTfa98xx"
#include "SpeakerProtectionTfa98xx.h"
#include <log/log.h>

// Define the static mutex
std::mutex SpeakerProtectionTfa98xx::calibrationMutex;

SpeakerProtectionTfa98xx::SpeakerProtectionTfa98xx()
    : isInitialized(false), speakerCount(0), powerAmpCount(0), isValidCalibration(false) {
    int status = 0;
    rm = ResourceManager::getInstance();
    if (!rm) {
        PAL_ERR(LOG_TAG, "Failed to get ResourceManager instance");
        return;
    }

    // Getting mixture controls from Resource Manager
    status = rm->getVirtualAudioMixer(&virtMixer);
    if (status) {
        PAL_ERR(LOG_TAG, "virt mixer error %d", status);
    }

    status = rm->getHwAudioMixer(&hwMixer);
    if (status) {
        PAL_ERR(LOG_TAG, "hw mixer error %d", status);
    }

    calibrationInfoInit();
    updateCalibrationValue();
    PAL_INFO(LOG_TAG, "SpeakerProtectionTfa98xx initialized");
    isInitialized = true;
}

SpeakerProtectionTfa98xx::~SpeakerProtectionTfa98xx() {
    hwMixer = nullptr;
    virtMixer = nullptr;
}

bool SpeakerProtectionTfa98xx::isTfaDevicePresent(struct mixer* hwMixer) {
    return mixer_get_ctl_by_name(hwMixer, "TFA Calibration");
}

void SpeakerProtectionTfa98xx::calibrationInfoInit() {
    calibratedImpedance = mixer_get_ctl_by_name(hwMixer, "TFA Calibration");
    if (!calibratedImpedance) {
        PAL_ERR(LOG_TAG, "Invalid mixer control: TFA Calibration");
        return;
    }

    speakerCount = mixer_ctl_get_num_values(calibratedImpedance);
    if (speakerCount <= 0) {
        PAL_ERR(LOG_TAG, "Invalid speaker count: %d", speakerCount);
        return;
    }

    // 2 speakers -> 1 power amp
    // 4 speakers -> 2 power amps
    powerAmpCount = std::min((speakerCount + 1) >> 1, static_cast<int>(MAX_PA_COUNT));

    PAL_INFO(LOG_TAG, "speakerCount:%d, powerAmpCount:%d", speakerCount, powerAmpCount);

    defaultImpedance = mixer_get_ctl_by_name(hwMixer, "TFA Default Impedance");
    if (!defaultImpedance) {
        PAL_ERR(LOG_TAG, "Invalid mixer control: TFA Default Impedance");
        return;
    }

    bool hasCalibrationNodes = false;
    caliInfo.clear();
    
    // Try to read calibration info from device nodes
    for (int i = 0; i < speakerCount; i++) {
        if (FILE* fp = openDeviceFile(DEVICE_ADDRESSES[i], "cali_info")) {
            hasCalibrationNodes = true;
            char buffer[64] = {0};
            if (readDeviceFile(buffer, sizeof(buffer), fp) > 0) {
                CaliInfo info = {};
                uint32_t addr;
                if (sscanf(buffer, "0x%x, %hhx, %d, %d\n", &addr, &info.dev_idx, &info.min_imp,
                           &info.max_imp) == 4) {
                    info.i2c_addr = DEVICE_ADDRESSES[i];
                    if (info.dev_idx < speakerCount) {
                        caliInfo.push_back(info);
                    }
                }
            }
            fclose(fp);
        }
    }

    if (!hasCalibrationNodes) {
        PAL_INFO(LOG_TAG, "No calibration nodes found, using hardcoded impedance values");
        for (int i = 0; i < speakerCount; i++) {
            CaliInfo info = {};
            info.dev_idx = i;
            info.i2c_addr = DEVICE_ADDRESSES[i];
            info.min_imp = DEFAULT_MIN_IMPEDANCE;
            info.max_imp = DEFAULT_MAX_IMPEDANCE;
            caliInfo.push_back(info);
        }
    }

    if (!caliInfo.empty()) {
        std::sort(caliInfo.begin(), caliInfo.end());

        for (auto& info : caliInfo) {
            info.def_imp = mixer_ctl_get_value(defaultImpedance, info.dev_idx);
            PAL_INFO(LOG_TAG, "Speaker %hhx: addr=0x%x, impedance:(min=%dmΩ, max=%dmΩ, default=%dmΩ)",
                     info.dev_idx, info.i2c_addr, info.min_imp, info.max_imp, info.def_imp);
        }
    }
}

void SpeakerProtectionTfa98xx::updateCalibrationValue() {
    isValidCalibration = false;
    struct mixer_ctl* calCtl = mixer_get_ctl_by_name(hwMixer, "TFA Calibration");
    if (!calCtl) return;

    for (auto& info : caliInfo) {
        int calValue = mixer_ctl_get_value(calCtl, info.dev_idx);
        info.cal_imp = calValue;
        // DSP impedance is represented in ohms (Ω) in Q16.16 fixed-point format
        if (calValue > info.min_imp && calValue < info.max_imp) {
            info.dsp_imp = (calValue << 16) / 1000;
            isValidCalibration = true;
        } else {
            // Use default or safe values if calibration is invalid
            if (info.def_imp > 0) {
                info.cal_imp = info.def_imp;
                info.dsp_imp = (info.def_imp << 16) / 1000;
            } else {
                // Use average of min and max as a fallback
                info.cal_imp = (info.max_imp + info.min_imp) >> 1;
                info.dsp_imp = (info.cal_imp << 16) / 1000;
            }
            PAL_INFO(LOG_TAG, "Using fallback impedance for i2c=0x%x, dev_idx=%hhx: cal_imp=%d",
                     info.i2c_addr, info.dev_idx, info.cal_imp);
        }
        
        PAL_INFO(LOG_TAG, "Speaker i2c=0x%x dev_idx=%i: impedance values: cal=%dmΩ, dsp=0x%x",
                info.i2c_addr, info.dev_idx, info.cal_imp, info.dsp_imp);
    }
}

// References: OplusSpeakerTfa98xx::payloadSPConfig and tfa98xx_adsp_send_calib_values() from
// tfa98xx_v6.c
void SpeakerProtectionTfa98xx::payloadSPConfig(uint8_t** payload, size_t* size, uint32_t miid) {
    struct apm_module_param_data_t* header = nullptr;
    uint8_t* payloadInfo = nullptr;
    size_t payloadSize = 0, padBytes = 0;

    *payload = nullptr;
    *size = 0;

    if (caliInfo.empty() || caliInfo.size() != speakerCount) {
        PAL_ERR(LOG_TAG, "Invalid calibration info size: %zu, expected: %d", caliInfo.size(),
                speakerCount);
        return;
    }

    // 11 bytes for calibration data (1 reserved + 10 used)
    payloadSize = sizeof(struct apm_module_param_data_t) + 11;
    padBytes = PAL_PADDING_8BYTE_ALIGN(payloadSize);

    payloadInfo = (uint8_t*)calloc(1, payloadSize + padBytes);
    if (!payloadInfo) {
        PAL_ERR(LOG_TAG, "Failed to allocate payload memory");
        return;
    }
    header = (struct apm_module_param_data_t*)payloadInfo;
    header->module_instance_id = miid;
    header->param_id = TFADSP_RX_SET_COMMAND;
    header->error_code = 0x0;
    header->param_size = payloadSize - sizeof(struct apm_module_param_data_t);
    uint8_t* bytes = payloadInfo + sizeof(struct apm_module_param_data_t);

    // Reserve bytes[0] to match kernel implementation
    bytes[1] = 0x00;
    bytes[2] = 0x81;
    bytes[3] = 0x05;

    // Process each speaker's calibration data
    for (const auto& info : caliInfo) {
        if (info.dsp_imp == 0) {
            PAL_ERR(LOG_TAG, "Invalid DSP impedance for speaker %d", info.dev_idx);
            free(payloadInfo);
            return;
        }

        // Calculate array index based on speaker index (4 for left, 7 for right)
        int baseIdx = (info.dev_idx == 0) ? 4 : 7;

        // Store impedance value in big-endian format
        bytes[baseIdx + 0] = (info.dsp_imp >> 16) & 0xFF;
        bytes[baseIdx + 1] = (info.dsp_imp >> 8) & 0xFF;
        bytes[baseIdx + 2] = info.dsp_imp & 0xFF;

        PAL_INFO(LOG_TAG, "Speaker %d: impedance=0x%x", info.dev_idx, info.dsp_imp);
    }

    // For mono case, copy primary channel data to secondary
    if (speakerCount == 1) {
        memcpy(&bytes[7], &bytes[4], 3);
    }

    *size = payloadSize + padBytes;
    *payload = payloadInfo;
}

FILE* SpeakerProtectionTfa98xx::openDeviceFile(uint8_t i2c_addr, const char* parameter) {
    char path[128] = {0};
    snprintf(path, sizeof(path), "/proc/tfa98xx-%x/%s", i2c_addr, parameter);
    FILE* fp = fopen(path, "r");
    if (!fp) {
        PAL_ERR(LOG_TAG, "Failed to open %s: %s", path, strerror(errno));
        return nullptr;
    }
    return fp;
}

long SpeakerProtectionTfa98xx::readDeviceFile(char* buffer, size_t size, FILE* fp) {
    if (!buffer || !fp || size == 0) return 0;
    size_t bytes = fread(buffer, 1, size - 1, fp);
    if (bytes > 0) buffer[bytes] = '\0';
    return bytes;
}

int32_t SpeakerProtectionTfa98xx::sendPcmIdAndMiidToDriver(uint32_t miid, int pcmId) {
    struct mixer_ctl *mixerCtl = nullptr;
    int ret = 0;

    mixerCtl = mixer_get_ctl_by_name(hwMixer, "SP PCMID");
    if (!mixerCtl) {
        PAL_ERR(LOG_TAG, "Invalid mixer control: SP PCMID");
        return -EINVAL;
    }
    ret = mixer_ctl_set_value(mixerCtl, 0, pcmId);
    if (ret) {
        PAL_ERR(LOG_TAG, "Failed to set PCM ID");
        return ret;
    }

    mixerCtl = mixer_get_ctl_by_name(hwMixer, "SP MIID");
    if (!mixerCtl) {
        PAL_ERR(LOG_TAG, "Invalid mixer control: SP MIID");
        return -EINVAL;
    }
    ret = mixer_ctl_set_value(mixerCtl, 0, miid);

    PAL_DBG(LOG_TAG, "Sent PCM ID: %d, MIID: 0x%x to driver", pcmId, miid);
    return ret;
}

int32_t SpeakerProtectionTfa98xx::spkrProtProcessingMode(bool flag)
{
    int ret = 0, dir = TX_HOSTLESS, flags, viParamId = 0;
    char mSndDeviceName_vi[128] = {0};
    uint8_t* payload = NULL;
    uint32_t devicePropId[] = {0x08000010, 1, 0x2};
    uint32_t miid = 0;
    bool isTxFeandBeConnected = true;
    size_t payloadSize = 0;
    struct pal_device device;
    struct pal_channel_info ch_info;
    struct pal_stream_attributes sAttr;
    struct pcm_config config;
    struct mixer_ctl *connectCtrl = NULL;
    struct audio_route *audioRoute = NULL;
    struct vi_r0t0_cfg_t r0t0Array[MAX_PA_COUNT];
    struct agmMetaData deviceMetaData(nullptr, 0);
    struct mixer_ctl *beMetaDataMixerCtrl = nullptr;
    FILE *fp;
    std::string backEndName, backEndNameRx;
    std::vector <std::pair<int, int>> keyVector;
    std::shared_ptr<ResourceManager> rm;
    std::ostringstream connectCtrlNameBeVI;
    std::ostringstream connectCtrlName;
    int numberOfChannels = 0;
    
    std::unique_lock<std::mutex> lock(calibrationMutex);
    
    PAL_DBG(LOG_TAG, "Flag %d", flag);
    
    if (flag) {
        // Enable speaker protection
        updateCalibrationValue();
        
        // Get resource manager instance
        rm = ResourceManager::getInstance();
        if (!rm) {
            PAL_ERR(LOG_TAG, "Failed to get resource manager instance");
            ret = -EINVAL;
            goto exit;
        }
        
        memset(&device, 0, sizeof(device));
        memset(&sAttr, 0, sizeof(sAttr));
        memset(&config, 0, sizeof(config));
        
        keyVector.clear();
        
        // Get VI device sound name
        ret = rm->getAudioRoute(&audioRoute);
        if (0 != ret) {
            PAL_ERR(LOG_TAG, "Failed to get the audio_route address status %d", ret);
            goto exit;
        }
        
        device.id = PAL_DEVICE_IN_VI_FEEDBACK;
        ret = rm->getSndDeviceName(device.id, mSndDeviceName_vi);
        if (0 != ret) {
            PAL_ERR(LOG_TAG, "Failed to obtain tx snd device name for %d", device.id);
            goto exit;
        }
        
        if (speakerCount == 1) {
            strlcat(mSndDeviceName_vi, "-mono-1", DEVICE_NAME_MAX_SIZE);
        }
        PAL_DBG(LOG_TAG, "get the audio route %s", mSndDeviceName_vi);
        
        // Get the backend name for VI device
        rm->getBackendName(device.id, backEndName);
        if (!strlen(backEndName.c_str())) {
            PAL_ERR(LOG_TAG, "Failed to obtain tx backend name for %d", device.id);
            goto exit;
        }
        
        // Get device configuration
        PayloadBuilder::getDeviceKV(device.id, keyVector);
        
        // Set calibration parameters based on number of channels
        numberOfChannels = speakerCount > 0 ? speakerCount : 1;
        
        if (numberOfChannels == 2) {
            keyVector.push_back(std::make_pair(SPK_PRO_VI_MAP, STEREO_SPKR));
        } else if (numberOfChannels == 1) {
            if (powerAmpCount == 2) {
                keyVector.push_back(std::make_pair(SPK_PRO_VI_MAP, LEFT_SPKR));
            } else {
                keyVector.push_back(std::make_pair(SPK_PRO_VI_MAP, RIGHT_SPKR));
            }
        } else {
            PAL_ERR(LOG_TAG, "Unsupported channel count: %d", numberOfChannels);
            ret = -EINVAL;
            goto exit;
        }
        
        // Generate device metadata
        SessionAlsaUtils::getAgmMetaData(keyVector, keyVector, 
                                     (struct prop_data *)devicePropId, deviceMetaData);
        if (!deviceMetaData.size) {
            PAL_ERR(LOG_TAG, "VI device metadata is zero");
            ret = -EINVAL;
            goto exit;
        }
        
        // Configure VI device with metadata
        connectCtrlNameBeVI << backEndName << " metadata";
        beMetaDataMixerCtrl = mixer_get_ctl_by_name(virtMixer,
                                           connectCtrlNameBeVI.str().data());
        if (!beMetaDataMixerCtrl) {
            PAL_ERR(LOG_TAG, "invalid mixer control for VI: %s", backEndName.c_str());
            ret = -EINVAL;
            goto exit;
        }
        
        ret = mixer_ctl_set_array(beMetaDataMixerCtrl, deviceMetaData.buf, deviceMetaData.size);
        free(deviceMetaData.buf);
        deviceMetaData.buf = nullptr;
        
        // Configure media format for device
        ret = SessionAlsaUtils::setDeviceMediaConfig(rm, backEndName, &device);
        if (ret) {
            PAL_ERR(LOG_TAG, "setDeviceMediaConfig for feedback device failed");
            goto exit;
        }
        
        /* Retrieve Hostless PCM device id */
        sAttr.type = PAL_STREAM_LOW_LATENCY;
        sAttr.direction = PAL_AUDIO_INPUT_OUTPUT;
        dir = TX_HOSTLESS;
        std::vector<int> pcmDevIds = rm->allocateFrontEndIds(sAttr, dir);
        if (pcmDevIds.size() == 0) {
            PAL_ERR(LOG_TAG, "allocateFrontEndIds failed");
            ret = -ENOSYS;
            goto exit;
        }
        
        connectCtrlName << "PCM" << pcmDevIds.at(0) << " connect";
        connectCtrl = mixer_get_ctl_by_name(virtMixer, connectCtrlName.str().data());
        if (!connectCtrl) {
            PAL_ERR(LOG_TAG, "invalid mixer control: %s", connectCtrlName.str().data());
            ret = -EINVAL;
            if (pcmDevIds.size() != 0) {
                rm->freeFrontEndIds(pcmDevIds, sAttr, dir);
                pcmDevIds.clear();
            }
            goto exit;
        }
        
        ret = mixer_ctl_set_enum_by_string(connectCtrl, backEndName.c_str());
        if (ret) {
            PAL_ERR(LOG_TAG, "Mixer control %s set with %s failed: %d",
                    connectCtrlName.str().data(), backEndName.c_str(), ret);
            if (pcmDevIds.size() != 0) {
                rm->freeFrontEndIds(pcmDevIds, sAttr, dir);
                pcmDevIds.clear();
            }
            goto exit;
        }
        
        // Configure PCM for VI path
        switch (24) { // Assuming 24-bit for TFA98xx
            case 32:
                config.format = PCM_FORMAT_S32_LE;
                break;
            case 24:
                config.format = PCM_FORMAT_S24_LE;
                break;
            case 16:
                config.format = PCM_FORMAT_S16_LE;
                break;
            default:
                PAL_DBG(LOG_TAG, "Unsupported bit width. Set default as 16");
                config.format = PCM_FORMAT_S16_LE;
                break;
        }
        
        config.rate = 48000; // Assuming 48kHz for TFA98xx
        config.channels = numberOfChannels;
        config.period_size = 240;
        config.period_count = 4;
        config.start_threshold = 0;
        config.stop_threshold = INT_MAX;
        config.silence_threshold = 0;
        
        flags = PCM_IN;
        
        // Read calibration values and create payload
        payloadSPConfig(&payload, &payloadSize, miid);
        if (payloadSize) {
            // Send calibration data to driver
            ret = sendPcmIdAndMiidToDriver(miid, pcmDevIds.at(0));
            if (ret) {
                PAL_ERR(LOG_TAG, "Failed to send PCM ID and MIID to driver");
                free(payload);
                if (pcmDevIds.size() != 0) {
                    rm->freeFrontEndIds(pcmDevIds, sAttr, dir);
                    pcmDevIds.clear();
                }
                goto exit;
            }
            
            free(payload);
        }
        
        // Apply audio route for VI path
        if (audioRoute) {
            audio_route_apply_and_update_path(audioRoute, mSndDeviceName_vi);
        }
        
        // Open PCM device for VI path
        struct pcm *txPcm = pcm_open(rm->getVirtualSndCard(), pcmDevIds.at(0), flags, &config);
        if (!txPcm) {
            PAL_ERR(LOG_TAG, "txPcm open failed");
            if (pcmDevIds.size() != 0) {
                disconnectFeandBe(pcmDevIds, backEndName);
                rm->freeFrontEndIds(pcmDevIds, sAttr, dir);
                pcmDevIds.clear();
            }
            goto exit;
        }
        
        if (!pcm_is_ready(txPcm)) {
            PAL_ERR(LOG_TAG, "txPcm open not ready");
            pcm_close(txPcm);
            if (pcmDevIds.size() != 0) {
                disconnectFeandBe(pcmDevIds, backEndName);
                rm->freeFrontEndIds(pcmDevIds, sAttr, dir);
                pcmDevIds.clear();
            }
            goto exit;
        }
        
        PAL_DBG(LOG_TAG, "pcm start for TX");
        if (pcm_start(txPcm) < 0) {
            PAL_ERR(LOG_TAG, "pcm start failed for TX path");
            pcm_close(txPcm);
            if (pcmDevIds.size() != 0) {
                disconnectFeandBe(pcmDevIds, backEndName);
                rm->freeFrontEndIds(pcmDevIds, sAttr, dir);
                pcmDevIds.clear();
            }
            goto exit;
        }
        
    } else {
        // Disable speaker protection
        PAL_DBG(LOG_TAG, "Closing VI path");
        
        // Get resource manager instance
        rm = ResourceManager::getInstance();
        if (!rm) {
            PAL_ERR(LOG_TAG, "Failed to get resource manager instance");
            ret = -EINVAL;
            goto exit;
        }
        
        device.id = PAL_DEVICE_IN_VI_FEEDBACK;
        
        ret = rm->getAudioRoute(&audioRoute);
        if (0 != ret) {
            PAL_ERR(LOG_TAG, "Failed to get the audio_route address status %d", ret);
            goto exit;
        }
        
        ret = rm->getSndDeviceName(device.id, mSndDeviceName_vi);
        if (0 != ret) {
            PAL_ERR(LOG_TAG, "Failed to obtain tx snd device name for %d", device.id);
            goto exit;
        }
        
        if (speakerCount == 1) {
            strlcat(mSndDeviceName_vi, "-mono-1", DEVICE_NAME_MAX_SIZE);
        }
        
        rm->getBackendName(device.id, backEndName);
        if (!strlen(backEndName.c_str())) {
            PAL_ERR(LOG_TAG, "Failed to obtain tx backend name for %d", device.id);
            goto exit;
        }
        
        // TODO: Add code to stop and close PCM, free resources based on actual state tracking
        if (audioRoute) {
            audio_route_reset_and_update_path(audioRoute, mSndDeviceName_vi);
        }
    }
    
exit:
    return ret;
}

// Private helper to disconnect frontend and backend
void SpeakerProtectionTfa98xx::disconnectFeandBe(std::vector<int> pcmDevIds, std::string backEndName) {
    if (pcmDevIds.size() == 0)
        return;
            
    std::ostringstream disconnectCtrlName;
    disconnectCtrlName << "PCM" << pcmDevIds.at(0) << " connect";
    
    struct mixer_ctl *disconnectCtrl = NULL;
    
    disconnectCtrl = mixer_get_ctl_by_name(virtMixer, disconnectCtrlName.str().data());
    if (!disconnectCtrl) {
        PAL_ERR(LOG_TAG, "invalid mixer control: %s", disconnectCtrlName.str().data());
        return;
    }
    
    mixer_ctl_set_enum_by_string(disconnectCtrl, "ZERO");
    PAL_DBG(LOG_TAG, "Backend %s is disconnected from front end", backEndName.c_str());
    
    // Don't free the frontend IDs here as it's already being done in the calling context
}