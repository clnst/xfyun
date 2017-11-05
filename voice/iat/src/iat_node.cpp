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

#include <iat/iat_node.h>

bool flag = false;
bool recorder_Flag = true;
static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;
string result = "";


void show_result(char *str, char is_over)
{
    printf("\rResult: [ %s ]", str);
    if(is_over)
        putchar('\n');
    string s(str);
    result = s;
    flag = true;                        //设置发布话题为真
}

void on_result(const char *result, char is_last)
{
    if (result) {
        size_t left = g_buffersize - 1 - strlen(g_result);
        size_t size = strlen(result);
        if (left < size) {
            g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
            if (g_result)
                g_buffersize += BUFFER_SIZE;
            else {
                printf("mem alloc failed\n");
                return;
            }
        }
        strncat(g_result, result, size);
        show_result(g_result, is_last);
    }
}

void on_speech_begin()
{
    if (g_result)
    {
        free(g_result);
    }
    g_result = (char*)malloc(BUFFER_SIZE);
    g_buffersize = BUFFER_SIZE;
    memset(g_result, 0, g_buffersize);
 
    printf("Start Listening...\n");
}

void on_speech_end(int reason)
{
    if (reason == END_REASON_VAD_DETECT)
    {
        printf("\nSpeaking done \n");
        recorder_Flag = false;
    }
    else
        printf("\nRecognizer error %d\n", reason);
}

void Iat::record(const char* session_begin_params)
{
    int errcode;
    int i = 0;
 
    struct speech_rec iat;
 
    struct speech_rec_notifier recnotifier = {
        on_result,
        on_speech_begin,
        &on_speech_end
    };
 
    errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
    if (errcode) {
        printf("speech recognizer init failed\n");
        return;
    }
    errcode = sr_start_listening(&iat);
    if (errcode) {
        printf("start listen failed %d\n", errcode);
    }
    /* demo 15 seconds recording */
    while(recorder_Flag)
    {
       sleep(1);
    }
    errcode = sr_stop_listening(&iat);
    if (errcode) {
        printf("stop listening failed %d\n", errcode);
    }
 
    sr_uninit(&iat);
}

void Iat::iat_cb(const std_msgs::Int32::ConstPtr &msg)
{
    ROS_INFO_STREAM("Topic is Subscriber");
    if(msg->data == ASRFLAG)
    {
        //iat_proc();
        record(session_begin_params.c_str());
    }
}

Iat::Iat(void)
{
    //Init node
    ROS_ASSERT(init());
}

Iat::~Iat(void)
{
    MSPLogout(); //退出登录
}

bool Iat::init(void)
{
    int ret = MSP_SUCCESS;

    ros::param::get("~sub_topic", sub_topic);
    ros::param::get("~pub_topic", pub_topic);

    ros::param::get("~user_id", user_id);

    ros::param::get("~sub", sub_value);
    ros::param::get("~domain", domain_value);
    ros::param::get("~language", language_value);
    ros::param::get("~accent", accent_value);
    ros::param::get("~sample_rate", sample_rate_value);
    ros::param::get("~result_type", result_type_value);
    ros::param::get("~result_encoding", result_encoding_value);


    login_params = "appid = "+ user_id +", work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动

    session_begin_params =
          "sub = " +sub_value+\
          ", domain = " +domain_value+\
          ", language = " +language_value+\
          ", accent = " +accent_value+\
          ", sample_rate = " +sample_rate_value+\
          ", result_type = " +result_type_value+\
          ", result_encoding = " + result_encoding_value;

    ret = MSPLogin(NULL, NULL, login_params.c_str());
    if(MSP_SUCCESS == ret)
    {
        iat_sub = nh.subscribe(sub_topic, 1, &Iat::iat_cb, this);
        iat_pub = nh.advertise<std_msgs::String>(pub_topic, 3);
    }else{
        printf("MSPLogin failed, error code: %d.\n", ret);
        MSPLogout(); //退出登录
        return false;
    }
    return true;
}

bool Iat::update(void)
{
    if(flag)
    {
        std_msgs::String msg;
        msg.data = result;
        iat_pub.publish(msg);
        flag = false;
        recorder_Flag = true;
    }
    ros::spinOnce();
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "xf_iat");
    Iat iat;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        iat.update();
        loop_rate.sleep();
    }

    return 0;
}
