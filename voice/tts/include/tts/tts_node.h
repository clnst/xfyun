/*******************************************************************************
* Copyright 2017 CLNST CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef __TTS_NODE_H__
#define __TTS_NODE_H__

#include <ros/ros.h>
#include <ros/time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <std_msgs/String.h>

#include "tts/qtts.h"
#include "tts/msp_cmn.h"
#include "tts/msp_errors.h"

class Tts{
public:
    Tts(void);
    ~Tts(void);

    bool init(void);
    bool update(void);

private:
    /* wav音频头部格式 */
    typedef struct _wave_pcm_hdr
    {
        char        riff[4];                // = "RIFF"
        int         size_8;                 // = FileSize - 8
        char        wave[4];                // = "WAVE"
        char        fmt[4];                 // = "fmt "
        int         fmt_size;               // = 下一个结构体的大小 : 16

        short int   format_tag;             // = PCM : 1
        short int   channels;               // = 通道数 : 1
        int         samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
        int         avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
        short int   block_align;            // = 每采样点字节数 : wBitsPerSample / 8
        short int   bits_per_sample;        // = 量化比特数: 8 | 16

        char        data[4];                // = "data";
        int         data_size;              // = 纯数据长度 : FileSize - 44
    } wave_pcm_hdr;

    ros::NodeHandle nh;
    ros::Subscriber tts_sub;

    std::string sub_topic;
    std::string login_params;
    std::string session_begin_params;
    std::string tmp_file;

    std::string user_id;

    std::string voice_name_value;
    std::string text_encoding_value;
    std::string sample_rate_value;
    std::string speed_value;
    std::string volume_value;
    std::string pitch_value;
    std::string rdn_value;

    /* 默认wav音频头部数据 */
    wave_pcm_hdr default_wav_hdr;

    int text_to_speech(const char* src_text, const char* des_path, const char* params);
    void tts_cb(const std_msgs::String::ConstPtr &msg);
    int play(const char* text);
};


#endif // __TTS_NODE_H__