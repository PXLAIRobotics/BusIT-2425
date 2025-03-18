# BusIT 2024-2025

## Overview & Course materials
| Week             | Main Topic      | Sub Topic(s)                                                         |
|------------------|-----------------|--------------------------------------------------------------------|
| [Day 1](#day-1-intro) | Intro           | Introduction to Robotics, Introduction to ROS2                            |
| [Day 2](#day-2-computer-vision) | Computer Vision    | Computer Vision, Project  |
| [Day 3](#day-3-lidar) | LiDAR           | LiDAR, Project  |
| [Day 4-5](#day-4-5-workshop--project) | Workshop & Project           |  |


**Planning:** [link](https://docs.google.com/spreadsheets/d/15HdELTIhPZTT3MXOrwgG9T98d1KogXK6/edit?usp=sharing&ouid=113903584216099729669&rtpof=true&sd=true)

## Teams
[Link](https://docs.google.com/spreadsheets/d/1dHRB9edoT9GNURKBkNpozU5cAfWZmviYu-xNyaK5ZBU/edit?usp=sharing)

## Development environment

Repository: https://github.com/PXLAIRobotics/ROS2JazzyDocker

## Discord server

Invite link: https://discord.gg/Nnt9ps3h

---

## Day 1: Intro

### Introduction
1. [01 - Introduction to the international BusIT week](https://docs.google.com/presentation/d/12ZiFGHrbpHBhwVBuW8IWCm55SUOD1VUCmkWGArjm-QY/edit?usp=sharing)
2. [02 - Introduction to robotics](https://docs.google.com/presentation/d/1Ea8KgwEG3dnj2lr9Qx2mFDlJjXus8Dv746eq6JwCrKM/edit?usp=sharing)

### ROS
1. [01 - Introduction to ROS](https://docs.google.com/presentation/d/1A70W-VW3kemfXvujr_nldWl8DQj6SrUCwiprwCcMDIs/edit?usp=sharing)
2. [02 - ROS Concepts](https://docs.google.com/presentation/d/1TEgNU2JJWePJjYKhwV_uKXRS158xJwDBQ-iIbaIBwY4/edit?usp=sharing)
3. [03 - ROS: Publisher Subscriber](https://docs.google.com/presentation/d/1G5NH5ieQ07DK02CBGnmbQ-mWS0jAgumsJW_Y5fxJBIE/edit?usp=sharing)
4. [04 - Simulation Environments](https://docs.google.com/presentation/d/1Noik0XNZwpObY1Dc9F9luQUJamzDhPwhPbc9501UQfg/edit?usp=sharing)

### Exercises

1. Start the gazebo sim

```
gz sim
```

...and start the Prius on Sonoma Raceway world.

2. Create a ROS bridge for the `/cmd_vel` topic

```
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

3. Create a Teleop Node that allows you to control the Prius using keyboard inputs.

4. Always remember to rebuild and source each time you change anything:

```
clean_and_rebuild_ros2_workspace
```

If that does not work, you can do it manually:

```
colcon build
source install/setup.bash
```

5. Go nuts


## Day 2: Computer Vision

### OpenCV
1. [01 - An introduction to OpenCV](https://docs.google.com/presentation/d/1WvdhUHXD4450N8q-2Ki4GRC-GsHrZoYAnINF3KHq4qE/edit?usp=sharing)

### Exercises

1. Start the gazebo sim

```
gz sim
```

...and start the Prius on Sonoma Raceway world.

2. Create a ROS bridge for the `/front_camera` topic

```
ros2 run ros_gz_bridge parameter_bridge /front_camera@sensor_msgs/msg/Image@gz.msgs.Image
```

3. Create a vision_controller package:

```
cd ~/Projects/ros2_workspace/src
ros2 pkg create --build-type ament_python vision_controller --dependencies rclpy image_transport cv_bridge sensor_msgs std_msgs opencv2
```

5. Add the `src/process_front_camera.py` script

6. Always remember to rebuild and source each time you change anything:

```
clean_and_rebuild_ros2_workspace
```

If that does not work, you can do it manually:

```
colcon build
source install/setup.bash
```

7. Drive autonomously along the racetrack. Use the Example OpenCV code for inspiration.


## Day 3: LiDAR

### OpenCV


### Exercises


## Day 4-5: Workshop & Project

Project presentation: https://docs.google.com/presentation/d/1IvJ7ySwOn4rOBmr-oBQ5eRKsJNSuDHAO_sBHsyaPHCQ/edit?usp=sharing

It's recommended to use a Miro board for your business idea (https://miro.com/) to collaborate easily.

### Additional info
How to get an overview of all gazebo topics:

```
gz topic -l
```

Then you can use a ROS-Gazebo bridge for whatever topic you need.

---

## Troubleshooting

### 1.

`ros2 run package_1 publisher` output: 

> Package 'package_1' not found

**Solution:** You forgot to...

```
cd ~/Projects/ros2_workspace
colcon build
source install/setup.bash
```

### 2.

`ros2 run package_1 subscriber` output: 

> No executable found


**Solution:** You forgot to add subscriber to `setup.py`.


### 3.

`colcon build` output:

> Summary: 0 packages finished [0.97s]
  1 package failed: package_1
  1 package aborted: example_package
  1 package had stderr output: package_1
Command '['/usr/bin/python3', '-c', 'import sys;from contextlib import suppress;exec("with suppress(ImportError):    from setuptools.extern.packaging.specifiers    import SpecifierSet");exec("with suppress(ImportError):    from packaging.specifiers import SpecifierSet");from distutils.core import run_setup;dist = run_setup(    \'setup.py\', script_args=(\'--dry-run\',), stop_after=\'config\');skip_keys = (\'cmdclass\', \'distclass\', \'ext_modules\', \'metadata\');data = {    key: value for key, value in dist.__dict__.items()     if (        not key.startswith(\'_\') and         not callable(value) and         key not in skip_keys and         key not in dist.display_option_names    )};data[\'metadata\'] = {    k: v for k, v in dist.metadata.__dict__.items()     if k not in (\'license_files\', \'provides_extras\')};sys.stdout.buffer.write(repr(data).encode(\'utf-8\'))']' returned non-zero exit status 1.
	
**Solution:** You forgot a comma in your `setup.py` file

---

## Increasing Docker container performance (on Windows)

### 1.

Create a .wslconfig file in this location:
`C:\Users\<UserName>\.wslconfig`

### 2.

Open the created .wslconfig file (with Notepad or VSCode), change your preferred memory usage for the container and number of logical processors:

> [!NOTE]  
> The default settings for WSL are:
>
> - Memory: 50% of total memory on Windows.
> - Processors: The same number of logical processors on Windows.
>
> So to increase performance you want to set this to the maximum number of logical processors and more than 50% of the memory on your host device, otherwise the container will get less resources than before.
>
> [Source (and more settings)](https://learn.microsoft.com/en-us/windows/wsl/wsl-config#main-wsl-settings)

```text
[wsl2]
memory=16GB
processors=8
```

### 3.

Shutdown WSL in a terminal window:

```
wsl --shutdown
```

### 4.

Restart Docker desktop.

### 5.

Start the container again.

### 6.

You can check if it works by entering `docker stats` in a terminal window.

---

## Additional resources

* ROS2 (Jazzy): https://docs.ros.org/en/jazzy/Tutorials.html
* Gazebo: https://gazebosim.org/docs/harmonic/tutorials/
* Gazebo x ROS2: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge
