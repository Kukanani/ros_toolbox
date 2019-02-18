#!/bin/bash
for (( ; ; ))
do
  rostopic pub -1 /gripper/cmd robotiq_85_msgs/GripperCmd "{emergency_release: false, emergency_release_dir: 0, stop: false, position: 0.0, speed: 128.0, force: 0.0}"
  rostopic pub -1 /gripper/cmd robotiq_85_msgs/GripperCmd "{emergency_release: false, emergency_release_dir: 0, stop: false, position: 1.0, speed: 255.0, force: 0.0}"
  rostopic pub -1 /gripper/cmd robotiq_85_msgs/GripperCmd "{emergency_release: false, emergency_release_dir: 0, stop: false, position: 0.0, speed: 255.0, force: 255.0}"
  rostopic pub -1 /gripper/cmd robotiq_85_msgs/GripperCmd "{emergency_release: false, emergency_release_dir: 0, stop: false, position: 1.0, speed: 255.0, force: 0.0}"
done