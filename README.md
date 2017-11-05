# xfyun
xfyun for ros

# Instruction

- Software tools:     PyCharm(2017+)

- Hardware tools:     MacBook Pro(2015)

- Operating system:   Ubuntu 16.04

- Ros version:        kinetic


# Documents

- http://doc.xfyun.cn/msc_linux/290898


# Install applications for xf_voice

- sudo apt-get install sox


# NLP support packages

- nlp(tuling)


# Ros subscriber topic

- "/xf_voice/tts"
- "/xf_voice/iat"


# Ros Publisher topic

- "/nlp/tuling"


# Usage method

- roslaunch tts xf_tts.launch
- roslaunch iat xf_iat.launch
- roslaunch nlp tl_nlp.launch


# Ros test xfyun

- rostopic pub -1 /xf_voice/iat std_msgs/Int32 1
