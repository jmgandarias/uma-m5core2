# UMA m5Stack Core2

University of Malaga repository for the M5stack Core2 device. With an special focus on using it in ROS projects.

The M5Stack Core2 Documentation can be found [here](https://docs.m5stack.com/en/core/core2).

**Author:** [Juan M.Gandarias](https://jmgandarias.com)<br />
**Affiliation:** [University of Málaga](https://www.uma.es)<br />
**Maintainer:** Juan M. Gandarias, jmgandarias@uma.es

The work on this repository started during my postdoc period at [HRII](https://hri.iit.it), [IIT](https://iit.it).<br />
This repo won't exist without the help of the best IIT technicians [Mattia Leonori](https://github.com/mleonori), and [Pietro Balatti](https://github.com/pietrobalatti).


## Table of Contents
- [UMA m5Stack Core2](#uma-m5stack-core2)
  - [Table of Contents](#table-of-contents)
  - [Drivers](#drivers)
  - [Arduino IDE Setup](#arduino-ide-setup)
  - [Avoid Arduino IDE](#avoid-arduino-ide)
  - [Install UMAM5Core2 library](#install-umam5core2-library)
  - [ROSSerial WiFi](#rosserial-wifi)
  - [Troubleshooting](#troubleshooting)
    - [Error 1](#error-1)
    - [Error 2](#error-2)
    - [Error 3](#error-3)

## Drivers
<details>
<summary><em>click to expand</em></summary>

- Ubuntu Drivers for M5Stack Core2 can be downloaded [here](https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/drivers/CP210x_VCP_Linux.zip). 
Open a Terminal, navigate towards the folder where you downloaded the drivers and run:
```
unzip CP210x_VCP_Linux.zip -d CP210x_VCP_Linux
cd CP210x_VCP_Linux
sudo tar xzvf Linux_3.x.x_4.x.x_VCP_Driver_Source.tar.gz
sudo make
```

- Then you need to copy the drivers into the drivers folder running the following comands:
```
sudo cp cp210x.ko /lib/modules/<your_kernel_version>/kernel/drivers/usb/serial/
sudo insmod /lib/modules/<your_kernel_version>/kernel/drivers/usb/serial/usbserial.ko
sudo insmod cp210x.ko
```

- If you don't know what kernel are you using, run:
```
uname -r
```
</details>

## Arduino IDE Setup
<details>
<summary><em>click to expand</em></summary>

Create two folders in /home/$USER:

```
cd
mkdir Arduino
mkdir Arduino_IDE_folder
```

Install any version of the Arduino IDE above 1.8.15 (Arduino IDE 2.0 and above are not yet supported at 18/10/2022).
[Here](https://downloads.arduino.cc/arduino-1.8.19-linux64.tar.xz) you can download the version 1.8.19

Once downloaded, copy and decompress the content of the file in /home/$USER/Arduino_IDE_folder.

Then you can install 
```
cd Arduino_IDE_folder
sudo ./install.sh
```

Once installed, you can open it 
```
./arduino
```

Check the installation was done correctly:
- In the Arduino IDE go to File > Preferences
- The Sketchbook location should be: /home/$USER/Arduino
- If you go to the /home/$USER/Arduino, a folder "libraries" should have been created containing all the basic/default arduino libraries


To setup the Arduino IDE for m5Stack, visit [this link](https://docs.m5stack.com/en/quick_start/core2/arduino). 


After doing this, you can test the hello world example: Re-connect the M5StackCore2 (It must be switched on), re-open the Arduino IDE, and open and deploy the hello world script in Examples>hello_world_m5stack>hello_world_m5stack.ino

</details>

## Avoid Arduino IDE
<details>
<summary><em>click to expand</em></summary>

Although needed to upload .ino programs to the M5stack boards, Arduino IDE is not the best IDE for developing.
Hence, to avoid open/close it everytime you have to upload an script, you can use your preferred IDE (e.g., VSCode) to code your program and then upload it from the Linux terminal directly running the following command:
```
arduino --upload <program_name.ino> --port <connected_port>
```
Note that specifying the port is optional and it's only needed if you have more than one board connected.

If you want to open the serial monitor from the terminal, you can run:
```
stty -F /dev/ttyACM* <your_baudrate> raw -clocal -echo
cat /dev/ttyACM*
```
</details>

## Install UMAM5Core2 library
<details>
<summary><em>click to expand</em></summary>

Clone the repo inside the Arduino libraries folder.
```
cd ~/Arduino/libraries
git clone git@github.com:jmgandarias/uma-m5core2.git
```
Note that you'll only need to do this once. I.e., if you want to use the UMAM5Core2 library in a different project/catkin_ws, you don't need to include it everytime. But also note that if you remove the git folder from ~/Arduino/libraries, you'll have to create the link again.
Remember that if you (or someone else) has changed something in this repo that you'd like to include in your project, you must pull inside the folder 
~/Arduino/libraries/uma-m5core2. Doing git_pull_all in your catkin_ws will NOT pull the uma-m5core2 repo.

To use this library you also need to install the rosserial library. You can follow the instructions in [this link](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup).

</details>

## ROSSerial WiFi
<details>
<summary><em>click to expand</em></summary>

- First, upload the example <ros_communication_test.ino> to use rosserial through WiFi with the M5StackCore2.
- Open a terminal and launch a roscore
- Open another terminal and run: 
```
rosrun rosserial_python serial_node.py tcp
```
- Reset the m5stack
- Check that the WiFi is correctly connected and you can see the message: "waiting for messages" in the screen
- Open one terminal and run:
```
rostopic echo /toggle_button
```
- Now, every time you press the circle button in the middle of the m5, you should receive an empty message in this terminal.
- Open a new terminal and run:
```
rostopic pub /toggle_led stmsgs/Empty --once
```
- The m5 should have received an empty msg and the screen should have blinked
</details>

## Troubleshooting
<details>
<summary><em>click to expand</em></summary>

### Error 1
If when uploading/verifying a .ino you receive this error 
> <span style="color:orange">ModuleNotFoundError: No module named 'serial'</span>

You need to know that Serial is not included with Python. It is a package that you need to install separately.
You can install serial from the command line with:
```
pip install pyserial
```

---
### Error 2

If during "sudo make", you receive the error 
> <span style="color:orange"> flex: Command not found <span>

You need to install it
```
sudo apt install flex
```

The same way, if the error is 
> <span style="color:orange"> bison: Command not found <span>

You need to install it
```
sudo apt install bison
```

---
### Error 3
If during "sudo make", you receive the error 
> <span style="color:orange">initialization of 'void (\*)(struct usb_serial_port \*)' from incopatible pointer type 'int (\*)(struct usb_serial_port \*) [-Werror=incopatible-pointer-types]</span>

Modify in cp210x.c line 50 from:
```bash
static int cp210x_port_remove(struct usb_serial_port *);
```
to:
```bash
static void cp210x_port_remove(struct usb_serial_port *);
```

Modify in cp210x.c line 1382 from:
```bash
static int cp210x_port_remove(struct usb_serial_port *)
{
    ...
    return 0;
}
```
to:
```bash
static int cp210x_port_remove(struct usb_serial_port *)
{
    ...
    //return 0;
}
```

---
</details>
