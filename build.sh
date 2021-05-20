#!/bin/bash

curPath=`pwd`

if [ "$1" == "all" ]; then
  cpu=$(arch)

  if [ "$cpu" == "x86_64" ]; then
    export PATH=$curPath/../../prebuilts/gcc/linux-x86/arm/gcc-linaro-6.3.1-2017.05-x86_64_arm-linux-gnueabihf/bin/:$PATH
    export ARCH=arm
    export CROSS_COMPILE=$curPath/../../prebuilts/gcc/linux-x86/arm/gcc-linaro-6.3.1-2017.05-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
    echo arm tools path : $PATH
  elif [ "$cpu" == "armv7l" ]; then
    echo arch: armv7l
  fi
  
  echo build ...
  make clean all
  exit 0
fi

if [ "$1" != "install" ]; then
  echo usage: ./build.sh build, or ./build.sh install
  exit 1
fi

if [ "$1" == "install" ]; then
  make install && make install_conf
  exit 1
fi

# 以下需要手动操作命令
user="admin"
keyfile="sx1302-ssh-key.pub"

echo please, output key file $keyfile
ssh-keygen -t rsa

# 执行下述两个命令是为了避免在安装文件时输入用户密码
ssh-copy-id -i $keyfile $user@localhost

# 解决 ssh_exchange_identification: read: Connection reset by peer问题
# [root@localhost Desktop]# vi /etc/hosts.allow
# #########################
# sshd: ALL    ##允许所有ip主机均能连接本机
if [ $? != 0 ]; then
  echo <<end
  # 解决 ssh_exchange_identification: read: Connection reset by peer问题
  # [root@localhost Desktop]# vi /etc/hosts.allow
  # #########################
  # sshd: ALL    ##允许所有ip主机均能连接本机
end
  exit 1
fi
# 您应该在输入raspberry pi密码后安装所有程式

make install

make install_conf

# 将global_conf.json等可执行文件复制到bin文件夹中

# 使用指令读取网关ID

./chip_id

# echo gateway_ID:

echo 将网关ID加入global_conf.json档案中

# 将网关ID加入global_conf.json 档案中

# "gateway_ID": "0x0016c001ff1801e6",

# 如何再Linux系统下启动SX1302网关

# 进入Packet Forwarder档案目录后执行程序:

# cd ~/sx1302_hal/bin/

echo "start gateway ..."
echo usage: ./lora_pkt_fwd -c global_conf.json

# ./lora_pkt_fwd -c global_conf.json.sx1250.EU868
