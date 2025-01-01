import robot_upstart
j = robot_upstart.Job(name="nera_robot_ros")
j.symlink = True
j.add(package="nera_bot", filename="launch/launch_hardware.launch.py")
j.install()