# MECH3895 Project - Robotic Manipulation Planning

This repository contains the source code for the MECH3895 project "Robotic Manipulation Planning".  

Note: if Gazebo models need to be installed, Gazebo node may not launch correctly. Gazebo will take some time to load as it is downloading required models, once loaded the arm will probably just fall due to controller timeout. Resolved by relaunching after Gazebo finishes downloading models and loads up.  

## Third Party Packages

### gazebo_grasp_plugin
Cloned from [gazebo-pkgs](https://github.com/JenniferBuehler/gazebo-pkgs) repo by [Jennifer Buehler](https://github.com/JenniferBuehler). The master branch was cloned, commit baf0f033475c3a592efb0862079f3ff8392cadf6.  

This package is used as a workaround to limitations in Gazebo for grasping operations (objects tend to slip/get pushed away when a grasp is attempted). Adding very high friction to the Kinova fingers is also a workaround. For this project, both the plugin and custom friction values are used.

### Kinova packages
The [kinova_control](kinova_control/), [kinova_description](kinova_description/), and [kinova_gazebo](kinova_gazebo) packages are from the official [kinova-ros](https://github.com/Kinovarobotics/kinova-ros/tree/master) package by Kinova Robotics. The master branch was cloned, commit 99ac039028855eb9c1000a9c51b9c1544d5ef446.