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

Mostly needed on the M100 Nuc:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### coinhsl

```bash
sudo pip3 install meson
```

### 3.2 CASADI

Install:
```bash
sudo apt-get install coinor-libipopt-dev
```

Compile Casadi:
```bash
git clone https://github.com/casadi/casadi.git casadi
```

