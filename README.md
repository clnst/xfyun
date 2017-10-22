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


# Ros subscriber topic

- "/xf_voice/tts"

- rostopic pub -1 /xf_voice/tts std_msgs/String "test tts"


# Usage method

- roslaunch tts xf_tts.launch
