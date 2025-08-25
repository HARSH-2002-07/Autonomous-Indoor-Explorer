🚀 Autonomous Indoor Explorer

A ROS1 Noetic project for building and simulating an autonomous indoor exploration robot.

📌 Project Overview

This project is part of my robotics journey where I design and implement an indoor mobile robot capable of mapping, localization, and navigation.

Unlike my minor project, this time I have built both the robot and the simulation world from scratch, ensuring a complete hands-on experience with ROS1 and Gazebo.
The development is broken into stages for structured progress and better tracking.

🛠️ Tech Stack

ROS1 (Noetic) – middleware for robotics

Gazebo – simulation environment

RViz – visualization & navigation goals

Python (rospy) – scripting & automation

Git + GitHub – version control

📂 Repository Structure
major_project_ws/
│── src/
│   └── autonomous_explorer/   # Main package (robot, world, launch files)
│── build/
│── devel/
└── README.md

🔑 Features

✔️ Custom Gazebo world 🌍

✔️ Custom robot model (URDF + sensors) 🤖

✔️ Teleoperation support 🎮

✔️ Mapping & Localization 🗺️

✔️ Path Planning & Navigation 🚦

⏳ Automated Exploration (next phase)

📌 Development Stages

✅ Workspace setup & repo initialization

✅ Create custom Gazebo world

✅ Build robot model (URDF + sensors)

✅ Teleop control & testing

✅ SLAM integration

✅ Navigation stack integration

🔄 Final integration & documentation

🚦 How to Run

Clone this repo and build the workspace:

git clone git@github.com:HARSH-2002-07/Autonomous-Indoor-Explorer.git
cd Autonomous-Indoor-Explorer/major_project_ws
catkin_make
source devel/setup.bash


Then launch the simulation with:

roslaunch autonomous_explorer bringup.launch

👤 Author

Harsh Jain
🎓 B.Tech CSE (AI & ML)
🔹 Aspiring Robotics Engineer | ROS Developer | AI Enthusiast
