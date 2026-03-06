# Commonly used commands

## Macos/Linux
- List usb drivers

ls /dev/tty.*  // Mac
ls /dev/ttyUSB* // Raspbery

## SSH
- copy to raspberry pi over ssh

rsync -avz --include-from=".pi.include" --exclude-from=".pi.exclude" -e ssh ./ z@raspberrypi.local:/home/z/mr

- single file

scp simulations/controllers/mobile_robot/mdps/avoid_and_track/env.py z@raspberrypi.local:/home/z/mr/simulations/controllers/mobile_robot/mdps/avoid_and_track/env.py

# demo
python camera_tcp_feed.py
python lidar_script.py