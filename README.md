# Build Instructions

For building the F1TENTH car, use the detailed step-by-step instructions of the [F1TENTH documentation](https://f1tenth.readthedocs.io/en/foxy_test/). If you prefer visual guidance, there are several build videos from F1TENTH available on [YouTube](https://www.youtube.com/watch?v=iyOtTtlHcvw).

This following Bill of Materials (BOM) lists all products and materials required for assembling a F1TENTH car based on a NVIDIA Jetson Orin NX and Hokuyo UST 20-LX LiDAR: [BOM Orin NX.xlsx](BOM%20Orin%20NX.xlsx) | [BOM Orin NX.pdf](BOM%20Orin%20NX.pdf).

# Installation Guide 

For setting up the NVIDIA Jetson Orin NX with ROS2 Humble on your F1TENTH robot, follow the installation guide. 

## Flashing Jetpack 6.0

Start by flashing Jetpack 6.0 on your Jetson with [NVIDIA SDK Manager](https://developer.nvidia.com/sdk-manager).
Follow the steps in the [Installation Instructions](https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html) and force your Jetson device into **recovery mode**.


## Setting up WLAN

To fix possible WLAN issues after flashing Jetpack 6.0, refer to this recorded [Bash History](WLAN_FIX_BASH_HISTORY.txt) and this [Ubuntu Forum Thread](https://askubuntu.com/questions/990363/how-to-load-iwlwifi-driver).

```bash
# Show interface name of wifi adapter
sudo nmcli d      

# Turn wifi on
sudo nmcli r wifi on

# List wifi networks
sudo nmcli d wifi list

# Connect to a wifi network. 
# Replace SSID and PW with your network's SSID and password.
sudo nmcli d wifi connect SSID password PW

# Show IP adress
ip addr show dev wlan0
```

## Update Packages and Reboot

Ensure your system is up to date and perform a reboot to apply any necessary changes.

```bash
sudo apt update
sudo apt full-upgrade
sudo reboot
```

## Installing Essential Packages

Install basic tools and utilities.

```bash
sudo apt install git 
sudo apt install nano 
sudo apt install python3-pip
```

## Create Swapfile for Memory-Intensive Tasks

Create a swap file, especially useful for memory-intensive applications.

```bash
sudo fallocate -l 4G /var/swapfile
sudo chmod 600 /var/swapfile
sudo mkswap /var/swapfile
sudo swapon /var/swapfile
sudo bash -c 'echo "/var/swapfile swap swap defaults 0 0" >> /etc/fstab'
```

## Installing Logitech F710 Driver

Clone and install the driver for the Logitech F710 gamepad.

```bash
git clone https://github.com/jetsonhacks/logitech-f710-module
cd logitech-f710-module
./install-module.sh
```

## Setting up Bluetooth

```bash
# Install bluetooth package
sudo apt install bluetooth
/etc/init.d/bluetooth status
/etc/init.d/bluetooth start

# Adding bluetooth device
bluetoothctl         # open new inner shell
scan on 
pair MAC_ADDRESS
connect MAC_ADDRESS
disconnect MAC_ADDRESS
```

For detailed instructions refer to this [guide on using Bluetooth via terminal](https://www.baeldung.com/linux/bluetooth-via-terminal).

## Setting up DS4 Controller

Follow these instructions to set up the Sony Dualshock 4 controller. For detailed steps, visit [this guide](https://ros-developer.com/2017/12/14/ps4-controller-bluetooth-ubuntu/). Skip, if you use a Logitech F710 control device. 

```bash
# Install ds4drv
sudo pip install ds4drv

# Run ds4drv
sudo ds4drv

# Hold the PS button and share button on the controller.

# Open joystick testing tool
sudo apt-get install jstest-gtk
```

## Adding Udev Rules

1. **Hokuyo LiDAR (not necessary if LiDAR is connected via Ethernet**

   ```bash
   sudo -i
   nano /etc/udev/rules.d/99-hokuyo.rules
   ```
   Add this line to the file:

   ```
   KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="15d1", MODE="0666", GROUP="dialout", SYMLINK+="sensors/hokuyo"
   ```

2. **VESC Motor Controller**

   ```bash
   sudo -i
   nano /etc/udev/rules.d/99-vesc.rules
   ```
   Add this line to the file:

   ```
   KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"
   ```

3. **DS4 Controller**

   ```bash
   sudo -i
   nano /etc/udev/rules.d/99-ds4.rules
   ```
   Add this line to the file:

   ```
   KERNEL=="js[0-9]*", ACTION=="add", ATTRS{idVendor}=="1d6b", ATTRS{idProduct}=="09cc", SYMLINK+="input/joypad-f710"
   ```

4. **Update Udev Rules**

   ```bash
   sudo u
   devadm control --reload-rules
   sudo udevadm trigger
   ```

5. **Checking Device Info**
   
   If you want to add udev rules for other devices, use the device kernel to walk through the device attributes.

   ```bash
   sudo udevadm info --name=<KERNEL> --attribute-walk
   ```

## Setting up the ethernet connection for the LiDAR

Connect the Hokuyo LiDAR with the Jetson using an ethernet cable and ensure that the LidAR is powered on.
Open network settings in the Linux GUI on the Jetson. 
In the network settings, you should see your ethernet connection (e.g., Wired Connection 1).
Click the gear icon next to your ethernet connection to open the settings.
In the "Wired Settings" dialog, go to the IPv4 tab.
Change the method from ```Automatic (DHCP)``` to ```Manual``` and enter the following settings:

- Address: ```192.168.0.15```
- Netmask: ```255.255.255.0```
- Gateway: ```192.168.0.1```

Call the connection ```Hokuyo UST-20LX```.

Test the connection with:
```bash
ping 192.168.0.10
```

## Installing Docker

Set up Docker's apt repository.
```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

Install the docker packages.
```bash
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Verify that the Docker Engine installation is successful by running the hello-world image.
```bash
sudo docker run hello-world
```

After runnin the hello-world image, you should get the following output in terminal.
```bash
Unable to find image 'hello-world:latest' locally
latest: Pulling from library/hello-world
478afc919002: Pull complete 
Digest: sha256:53cc4d415d839c98be39331c948609b659ed725170ad2ca8eb36951288f81b75
Status: Downloaded newer image for hello-world:latest

Hello from Docker!
This message shows that your installation appears to be working correctly.

To generate this message, Docker took the following steps:
 1. The Docker client contacted the Docker daemon.
 2. The Docker daemon pulled the "hello-world" image from the Docker Hub.
    (arm64v8)
 3. The Docker daemon created a new container from that image which runs the
    executable that produces the output you are currently reading.
 4. The Docker daemon streamed that output to the Docker client, which sent it
    to your terminal.

To try something more ambitious, you can run an Ubuntu container with:
 $ docker run -it ubuntu bash

Share images, automate workflows, and more with a free Docker ID:
 https://hub.docker.com/

For more examples and ideas, visit:
 https://docs.docker.com/get-started/
```

Add Docker group and user
```bash
# Create docker group
sudo groupadd docker
# Add user
sudo usermod -aG docker $USER
newgrp docker
```

## Installing ROS2 Humble

Make sure you have a locale which supports UTF-8. If you are in a minimal environment (such as a docker container), the locale may be something minimal like POSIX. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

You will need to add the ROS 2 apt repository to your system.
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install the ROS 2 packages
```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

Set up automatically source ROS 2 in every new terminal session.
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

For more see the official [ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

## Installing Colcon

Install Colcon, a command-line tool to build ROS 2 packages.

```bash
sudo apt install python3-colcon-common-extensions
```

## Setting up the F1/10 Driver Stack

Initialize and build the F1/10 driver stack in your workspace.

```bash
cd $HOME && mkdir -p f1tenth_ws/src && cd f1tenth_ws
colcon build
cd src
git clone https://github.com/kutschl/f1tenth_system.git
cd f1tenth_system
git submodule update --init --force --remote
cd $HOME/f1tenth_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src -i -y --rosdistro humble
colcon build
```

