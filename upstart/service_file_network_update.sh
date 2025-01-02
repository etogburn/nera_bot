sudo cp /home/robot/robot_ws/nera_bot/upstart/nera_bot_ros.service /lib/systemd/system/nera_bot_ros.service

# [Unit]
# Description="myservice"
# After=network.target
# Wants=network.target
# After=network-online.target
# Wants=network-online.target

# [Service]
# Type=simple
# ExecStart=/usr/sbin/myservice-start
# Restart=on-failure # add this
# RestartSec=5
# SuccessExitStatus=0 129 200

# [Install]
# WantedBy=multi-user.target