Make sure that it is very easy to understand how to both compile and get the source code to run on your platform, including complete setup and build instructions of dependencies, such as openCV. These instructions are preferably written in the README file in the root folder of the project, using markdown. Include a picture and a short description of your finished project in this file as well).

It is not possible to commit empty folders to git. If you still want to commit an empty folder, a quite common technique to add files called .gitkeep in the empty folders.

Try to keep your git-repository clean. As a rule-of-thumb, do not check in files that can be generated from the code in the repository, e.g., compiled binaries. In principle, neither pdf-files which are generated from the TeX-code should be checked in. However, do check in final versions of your reports, presentations and all other documentation - these are the same documents that go into the shared git repo.

To help you to avoid checking in files that are not needed, the .gitignore contains rules for which files that will not be added to git by default. If you explicitly add a specific file, it will be added regardless of the rules in the .gitignore file.

# FOR STUDENTS - CRAZYFLIE PREP

## 1. Issues

- 

## 2. Info

- check ubuntu version:

```bash
lsb_release -a
```
No LSB modules are available.

Distributor ID: Ubuntu

Description:    Ubuntu 20.04.1 LTS

Release:        20.04

Codename:       focal

- check python version:

```bash
python3 --version
```

## 3. Preparations

### 3.1 Install ROS Noetic (the only version compatible with Ubuntu 20.04 LTS)

Mostly set up your keys etc.
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
Installation itself
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```

Before you can use ROS, you need to initialize rosdep - system dependencies for source you want to compile and is required to run some core components in ROS, and set up your enviroment.
```bash
sudo rosdep init
rosdep update
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

```bash
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### 3.2 If you have bugs

