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

#include <tts/tts_node.h>


int Tts::text_to_speech(const char* src_text, const char* des_path, const char* params)
{
    int          ret          = -1;
    FILE*        fp           = NULL;
    const char*  sessionID    = NULL;
    unsigned int audio_len    = 0;
    wave_pcm_hdr wav_hdr      = default_wav_hdr;
    int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

    if (NULL == src_text || NULL == des_path)
    {
        printf("params is error!\n");
        return ret;
    }
    fp = fopen(des_path, "wb");
    if (NULL == fp)
    {
        printf("open %s error.\n", des_path);
        return ret;
    }
    /* 开始合成 */
    sessionID = QTTSSessionBegin(params, &ret);
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSSessionBegin failed, error code: %d.\n", ret);
        fclose(fp);
        return ret;
    }
    ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSTextPut failed, error code: %d.\n",ret);
        QTTSSessionEnd(sessionID, "TextPutError");
        fclose(fp);
        return ret;
    }
    printf("正在合成 ...\n");
    fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
    while (1)
    {
        /* 获取合成音频 */
        const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
        if (MSP_SUCCESS != ret)
            break;
        if (NULL != data)
        {
            fwrite(data, audio_len, 1, fp);
            wav_hdr.data_size += audio_len; //计算data_size大小
        }
        if (MSP_TTS_FLAG_DATA_END == synth_status)
            break;
        printf(">");
        usleep(15*1000); //防止频繁占用CPU
    }//合成状态synth_status取值请参阅《讯飞语音云API文档》
    printf("\n");
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSAudioGet failed, error code: %d.\n",ret);
        QTTSSessionEnd(sessionID, "AudioGetError");
        fclose(fp);
        return ret;
    }
    /* 修正wav文件头数据的大小 */
    wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);

    /* 将修正过的数据写回文件头部,音频文件为wav格式 */
    fseek(fp, 4, 0);
    fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
    fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
    fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
    fclose(fp);
    fp = NULL;
    /* 合成完毕 */
    ret = QTTSSessionEnd(sessionID, "Normal");
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSSessionEnd failed, error code: %d.\n",ret);
    }

    return ret;
}

void Tts::tts_cb(const std_msgs::String::ConstPtr &msg)
{
    play(msg->data.c_str());
}

int Tts::play(const char* text)
{
    int ret = MSP_SUCCESS;
    std::string cmd= "play ";

    /* 文本合成 */
    printf("开始合成 ...\n");
    ret = text_to_speech(text, tmp_file.c_str(), session_begin_params.c_str());
    if(MSP_SUCCESS == ret)
    {
        printf("合成完毕\n");
        cmd = cmd + tmp_file;
        system(cmd.c_str());
    }
    else
    {
        printf("text_to_speech failed, error code: %d.\n", ret);
    }

    return ret;
}

Tts::Tts(void)
{
    //Init node
    ROS_ASSERT(init());
}

Tts::~Tts(void)
{
    const char* quit = " 退出hercules two ";
    play(quit);
    MSPLogout(); //退出登录
}

bool Tts::init(void)
{
    int ret = MSP_SUCCESS;
    const char* start = " 启动hercules two ";

    ros::param::get("~sub_topic", sub_topic);

    ros::param::get("~user_id", user_id);

    ros::param::get("~tmp_file", tmp_file);

    ros::param::get("~voice_name", voice_name_value);
    ros::param::get("~text_encoding", text_encoding_value);
    ros::param::get("~sample_rate", sample_rate_value);
    ros::param::get("~speed", speed_value);
    ros::param::get("~volume", volume_value);
    ros::param::get("~pitch", pitch_value);
    ros::param::get("~rdn", rdn_value);

    default_wav_hdr.riff[0] = 'R';
    default_wav_hdr.riff[1] = 'I';
    default_wav_hdr.riff[2] = 'F';
    default_wav_hdr.riff[3] = 'F';
    default_wav_hdr.size_8 = 0;
    default_wav_hdr.wave[0] = 'W';
    default_wav_hdr.wave[1] = 'A';
    default_wav_hdr.wave[2] = 'V';
    default_wav_hdr.wave[3] = 'E';
    default_wav_hdr.fmt[0] = 'f';
    default_wav_hdr.fmt[1] = 'm';
    default_wav_hdr.fmt[2] = 't';
    default_wav_hdr.fmt[3] = ' ';
    default_wav_hdr.fmt_size = 16;
    default_wav_hdr.format_tag = 1;
    default_wav_hdr.channels = 1;
    default_wav_hdr.samples_per_sec = 16000;
    default_wav_hdr.avg_bytes_per_sec = 32000;
    default_wav_hdr.block_align = 2;
    default_wav_hdr.bits_per_sample = 16;
    default_wav_hdr.data[0] = 'd';
    default_wav_hdr.data[1] = 'a';
    default_wav_hdr.data[2] = 't';
    default_wav_hdr.data[3] = 'a';
    default_wav_hdr.data_size = 0;

    login_params = "appid = "+ user_id +", work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动

    session_begin_params =
        "voice_name = "+voice_name_value+\
        ", text_encoding = "+text_encoding_value+\
        ", sample_rate = "+sample_rate_value+\
        ", speed = "+speed_value+\
        ", volume = "+volume_value+\
        ", pitch = "+pitch_value+\
        ", rdn = "+rdn_value;

    ret = MSPLogin(NULL, NULL, login_params.c_str());
    if(MSP_SUCCESS == ret)
    {
        play(start);
        tts_sub = nh.subscribe(sub_topic, 10, &Tts::tts_cb, this);
    }
    else
    {
        printf("MSPLogin failed, error code: %d.\n", ret);
        MSPLogout(); //退出登录
        return false;
    }

    return true;
}

bool Tts::update(void)
{
    return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
    ros::init(argc, argv,"xf_tts");
    Tts tts;

    tts.update();
    ros::spin();

    return 0;
}
