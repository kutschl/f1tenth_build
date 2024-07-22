# Build Instructions

For building the F1TENTH car, use the detailed step-by-step instructions of the [F1TENTH documentation](https://f1tenth.readthedocs.io/en/foxy_test/). If you prefer visual guidance, there are several build videos from F1TENTH available on [YouTube](https://www.youtube.com/watch?v=iyOtTtlHcvw).

This following Bill of Materials (BOM) lists all products and materials required for assembling a F1TENTH car based on a NVIDIA Jetson Orin NX and Hokuyo UST 20-LX Lidar: [BOM Orin NX.xlsx](BOM%20Orin%20NX.xlsx).

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

## Teamviewer for Remote Desktop

Instructions for installing Teamviewer (for remote desktop access) will be added later.

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

1. **Hokuyo Lidar**

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

## Installing ROS2 Humble

Follow the official [ROS2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

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

## Sourcing Bash

Automatically source ROS 2 and F1/10 workspace environments in every new terminal session.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source $HOME/f1tenth_ws/install/setup.bash" >> ~/.bashrc
```
