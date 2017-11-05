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

#ifndef __IAT_NODE_H__
#define __IAT_NODE_H__

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iat/qisr.h>
#include <iat/msp_cmn.h>
#include <iat/msp_errors.h>
#include <iat/speech_recognizer.h>

#define FRAME_LEN   640
#define BUFFER_SIZE 4096
#define ASRFLAG     1

using namespace std;

class Iat{
public:
    Iat(void);
    ~Iat(void);

    bool init(void);
    bool update(void);

private:
    ros::NodeHandle nh;
    ros::Subscriber iat_sub;
    ros::Publisher iat_pub;

    std::string sub_topic;
    std::string pub_topic;
    std::string login_params;
    std::string session_begin_params;

    std::string user_id;

    std::string sub_value;
    std::string domain_value;
    std::string language_value;
    std::string accent_value;
    std::string sample_rate_value;
    std::string result_type_value;
    std::string result_encoding_value;

    void record(const char* session_begin_params);
    void iat_cb(const std_msgs::Int32::ConstPtr &msg);
};

#endif // __TTS_NODE_H__
