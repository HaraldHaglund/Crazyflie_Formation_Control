Make sure that it is very easy to understand how to both compile and get the source code to run on your platform, including complete setup and build instructions of dependencies, such as openCV. These instructions are preferably written in the README file in the root folder of the project, using markdown. Include a picture and a short description of your finished project in this file as well).

It is not possible to commit empty folders to git. If you still want to commit an empty folder, a quite common technique to add files called .gitkeep in the empty folders.

Try to keep your git-repository clean. As a rule-of-thumb, do not check in files that can be generated from the code in the repository, e.g., compiled binaries. In principle, neither pdf-files which are generated from the TeX-code should be checked in. However, do check in final versions of your reports, presentations and all other documentation - these are the same documents that go into the shared git repo.

To help you to avoid checking in files that are not needed, the .gitignore contains rules for which files that will not be added to git by default. If you explicitly add a specific file, it will be added regardless of the rules in the .gitignore file.

# FOR STUDENTS - CRAZYFLIE PREP

## 0. How to install VS Code and LaTex in Ubuntu

To install Visual Studio Code in Ubuntu 20.04 via the terminal, follow these steps. 

```bash
sudo apt update
sudo apt install software-properties-common apt-transport-https wget
```

Microsoft GPG key...

```bash
wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
```

```bash
sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
```

```bash
sudo apt update
sudo apt install code
```

## 1. Issues

You need eduroam - connect to LU-Guest and then we set up eduroam by going inside the folder where this is located
in the terminal and running 

```bash
cd Download # for example
```

```bash
python3 eduroam-linux-Lund_University.py
```

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

If you do not have rosdep on your ubuntu,

```bash
sudo apt update
sudo apt install python3-rosdep
```
 
You have to initialize rosdep

```bash
sudo rosdep init
rosdep update
```

Add the the ROS environment by adding the following line to your `~/.bashrc` file.

```bash
source /opt/ros/noetic/setup.bash
```

Install, the catkin tools if you are missing them

```bash
sudo apt-get update
sudo apt-get install python3-catkin-tools
```

### 3.3 Create your catkin workspace

Creating a ROS workspace is a common first step when working with ROS (Robot Operating System). A workspace is essentially a directory (folder) where you can organize and build your ROS packages. In your home do the following,

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

```bash
catkin_init_workspace
```

Then you need to build the workspace

```bash
cd ~/catkin_ws
catkin build
```

Every new terminal you open will not automatically know about your ROS workspace. To add your workspace to the ROS environment, you need to source the `setup.bash` script in your workspace's `devel` directory.


```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Adding the source command to your `~/.bashrc` file ensures that every new terminal session will automatically source your workspace, making ROS aware of your packages.

In the home directory confirm that the ROS path is correct

```bash
echo $ROS_PACKAGE_PATH
```

## 4. Install cfclient - FOR CRAZYFLIE

```bash
sudo apt install git python3-pip libxcb-xinerama0 libxcb-cursor0
pip3 install --upgrade pip
```

```bash
pip3 install cfclient
```

### 4.1 If you have bugs

```bash
sudo apt update && sudo apt upgrade
```

Install PyQt6 Dependencies...

```bash
sudo apt install qt5-default pyqt5-dev pyqt5-dev-tools qttools5-dev-tools
```

Manual installation if there is still bugs.

```bash
pip3 install PyQt6  # (PyQt6==6.2.3) Example version, adjust based on compatibility
```

```bash
nano .bashrc
```

The following should be located at the end of your .bashrc (THE END OF YOUR BASHRC FILE BY NOW SHOULD HAVE THESE THREE LINES IF ONE IS MISSING ADD IT MANUALY AND SAVE, be careful not to ruin anything above)

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export PATH="$HOME/.local/bin:$PATH"
```

To close it and save it you will pres ctrl+x and pick Y (for yes) and then enter

```bash
source ~/.bashrc
```

### 4.2 USB permissions 


Check your usb ports by inserting a random usb and doing the following in the terminal, if the radio doesn't show up the following below must be added.

```bash
lsusb
```

Get the rules going,

```bash
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER
```

```bash
cd /etc/udev/rules.d
```

Once inside this folder please do the following create a file with the following content.

```bash
sudo touch 99-crazyradio.rules 
```

```bash
sudo chmod +x 99-crazyradio.rules 
```

```bash
sudo nano +x 99-crazyradio.rules 
```

99-crazyradio.rules (WHAT THE FILE NEEDS TO LOOK LIKE) - the content is given below

```bash
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
```

To close it and save it you will pres ctrl+x and pick Y (for yes) and then enter

This is then done back in the home directory,

```bash
cd 
```

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Finally do the following to reboot your machine/computer

```bash
sudo reboot
```