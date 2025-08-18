🚀 Autonomous Indoor Explorer

A ROS1 (Noetic) project for building and simulating an autonomous indoor exploration robot.

📌 Project Overview

This project is part of my robotics journey where I design and implement an indoor mobile robot capable of autonomous navigation and mapping.
Unlike my minor project, here I am building both the robot and the world from scratch, ensuring a complete hands-on experience with ROS1 and Gazebo.

The project is structured in stages to ensure smooth development and tracking of progress.

🛠️ Tech Stack

ROS1 Noetic

Gazebo (for simulation)

RViz (for visualization)

Python (rospy)

Git & GitHub (for version control)

📂 Repository Structure
major_project_ws/
│── src/
│   └── autonomous_explorer/   # Main package for robot + world + launch files
│── build/
│── devel/
└── README.md

🔑 Features (Planned)

✔️ Custom Gazebo world
✔️ Custom robot model (URDF + sensors)
✔️ Teleoperation support
✔️ Mapping & Localization
✔️ Path Planning & Autonomous Navigation

📌 Development Stages

Stage 1 – Workspace setup & repo initialization ✅

Stage 2 – Create custom Gazebo world 🌍

Stage 3 – Build robot model (URDF + sensors) 🤖

Stage 4 – Teleop control & testing 🎮

Stage 5 – SLAM integration 🗺️

Stage 6 – Navigation stack 🚦

Stage 7 – Final integration & documentation 📖

🚦 How to Run

Clone this repo and build the workspace:

git clone git@github.com:HARSH-2002-07/Autonomous-Indoor-Explorer.git
cd Autonomous-Indoor-Explorer/major_project_ws
catkin_make
source devel/setup.bash

👤 Author

Harsh Jain
🎓 B.Tech CSE (AI & ML)
🔹 Aspiring Robotics Engineer | ROS Developer | AI Enthusiast
