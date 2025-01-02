cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
ros2 run robot_upstart install nera_bot/launch/launch_hardware.launch.py --job nera_bot_ros --symlink --interface wlan0 --user robot --setup ~/robot_ws/install/setup.bash
sudo cp /home/robot/robot_ws/nera_bot/upstart/nera_bot_ros.service /lib/systemd/system/nera_bot_ros.service
sudo systemctl daemon-reload
sudo systemctl start nera_bot_ros