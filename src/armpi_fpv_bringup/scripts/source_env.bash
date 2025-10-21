#!/bin/bash
setopt no_nomatch

sudo chgrp i2c /dev/i2c-*
sudo chgrp gpio /dev/gpio*

ip=$(ip addr show wlan0 | grep -o 'inet [0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | grep -o [0-9].*)
if [ -z $ip ]; then
  ip=$(ip addr show eth0 | grep -o 'inet [0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | grep -o [0-9].*)
fi

export HOST_IP=$ip
export MASTER_IP=$ip
export ROS_MASTER_URI=http://$MASTER_IP:11311
export ROS_HOSTNAME=$HOST_IP

echo -e "ROS_HOSTNAME: \033[32m$ROS_HOSTNAME\033[0m"
echo -e "ROS_MASTER_URI: \033[32m$ROS_MASTER_URI\033[0m"

if [ $ZSH_VERSION ]; then
  . /opt/ros/noetic/setup.zsh
  . $HOME/armpi_fpv/devel/setup.zsh
elif [ $BASH_VERSION ]; then
  . /opt/ros/noetic/setup.bash
  . $HOME/armpi_fpv/devel/setup.bash
else
  . /opt/ros/noetic/setup.sh
  . $HOME/armpi_fpv/devel/setup.sh
fi
export DISPLAY=:0.0
exec "$@"
