#!/bin/bash

echo "Uploading to Blue ROV"

rsync -avPzL -e ssh  ./detect_cameras.sh ./setup_camera.sh ./robot/motor_data.csv ./robot/robot.toml $1 pi@192.168.2.2:~/mate/ &&
  ssh pi@192.168.2.2 "sudo systemctl stop docker.socket && killall -SIGINT mate-bin && sleep 0.5 && killall -9 mate-bin ; cd ~/mate/ && rm ./mate-bin ; mv ./$(basename $1) ./mate-bin && ./mate-bin & cp -p mate-bin ./$(basename $1)"
