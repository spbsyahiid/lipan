# Articubot ROS 2 Simulation (ROS 2 Jazzy + Gazebo Harmonic)

This repo documents the complete setup of a simulated ROS 2 robot using ROS 2 Jazzy, Gazebo Harmonic (`gz sim`), and RViz2 on a **clean install of Ubuntu 24.04 (WSL2)**.

## ✅ Prerequisites

- Ubuntu 24.04 (tested on WSL2)
- Installed:
  - ROS 2 Jazzy (`ros-jazzy-desktop`)
  - `gazebo-harmonic`
  - `xacro`, `ros_gz` packages
- Git and VSCode (optional but recommended)
- Your graphics must support `gz sim` through WSL or native Linux.

---

## 🧰 Setup Steps

### 1. Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install ros-jazzy-desktop
Add to ~/.bashrc:

bash
Copy
Edit
source /opt/ros/jazzy/setup.bash
Apply changes:

bash
Copy
Edit
source ~/.bashrc
2. Install gz sim (Gazebo Harmonic)
bash
Copy
Edit
sudo apt install gz-sim8 gz-gui8
Also needed:

bash
Copy
Edit
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-ros-gz
3. Create ROS 2 Workspace
bash
Copy
Edit
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
4. Clone Articubot Repo
bash
Copy
Edit
git clone https://github.com/SwollEngineAF/dua_lipan.git articubot_one
cd ~/dev_ws
5. Build the Workspace
bash
Copy
Edit
colcon build --symlink-install
source install/setup.bash
Add the following to ~/.bashrc if you want auto-source every new terminal:

bash
Copy
Edit
source ~/dev_ws/install/setup.bash
🚀 Launch the Simulation
After successful build, run:

bash
Copy
Edit
ros2 launch articubot_one launch_sim_simple.launch.py
If RViz2 and gz sim don’t show up automatically, you can launch them manually in new terminals:

bash
Copy
Edit
gz sim
rviz2
🛠 Troubleshooting
Missing packages: Install them using apt install ros-jazzy-<package-name>.

Permission errors pushing to GitHub: Make sure to update the remote URL using your own repo:

bash
Copy
Edit
git remote set-url origin https://github.com/SwollEngineAF/dua_lipan.git
Gazebo crashing or no GUI: Ensure your display is working properly with WSLg and OpenGL. You may need to set:

bash
Copy
Edit
export DISPLAY=:0
export LIBGL_ALWAYS_INDIRECT=1
Add those to ~/.bashrc too.

✅ Status
 Fresh ROS 2 Jazzy install on Ubuntu 24.04

 Gazebo Harmonic simulation working

 RViz2 visualization

 GitHub integration tested

✍️ Author
Syahiid Rasidi – @SwollEngineAF