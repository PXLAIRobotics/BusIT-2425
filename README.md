# BusIT 2024-2025

## Overview & Course materials
| Week             | Main Topic      | Sub Topic(s)                                                         |
|------------------|-----------------|--------------------------------------------------------------------|
| [Day 1](#day-1-intro) | Intro           | Introduction to Robotics, Introduction to ROS2                            |
| [Day 2](#day-2-computer-vision) | Computer Vision    | Computer Vision & LiDAR  |
| [Day 3-5](#day-3-5-workshop--project) | Workshop & Project           |  |


**Planning:** [link](https://docs.google.com/spreadsheets/d/15HdELTIhPZTT3MXOrwgG9T98d1KogXK6/edit?usp=sharing&ouid=113903584216099729669&rtpof=true&sd=true)


## Development environment

Repository: https://github.com/PXLAIRobotics/ROS2JazzyDocker

## Discord server

https://discord.gg/Nnt9ps3h

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

2. Create a ROS bridge for the `cmd_vel` topic

```
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

3. Create a Teleop Node that allows you to control the Prius using keyboard inputs.

4. Go nuts


## Day 2: Computer Vision




## Day 3-5: Workshop & Project



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
