cd /etc/udev/rules.d/
sudo touch local.rules
sudo nano local.rules

# ACTION=="add", KERNEL=="ttyACM0", MODE="0666"
# username ALL=(ALL:ALL) ALL to file /etc/sudoers 