# Guide to Moveit for Teleoperation

## Install Moveit
```sudo apt install ros-humble-moveit```

```sudo apt install ros-humble-moveit-servo```

## Build and Source
```colcon build```

```source install/setup.bash```

## Run the code

```ros2 launch teleop teleop.launch.py```

```ros2 launch moveit_spear servo.launch.py```

```ros2 launch moveit_spear demo.launch.py```

```ros2 service call /arm_servo/start_servo std_srvs/srv/Trigger {}```



Refer to this for control mapping: https://drive.google.com/file/d/1n3aWn4sJ1-WZIfTOjwfjbIYzQQzIuocr/view?usp=drive_link


