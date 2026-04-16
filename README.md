sudo usermod -a -G dialout $USER

sudo apt install python3-serial


sudoedit /etc/udev/rules.d/50-myusb.rules
KERNEL=="ttyUSB[0-9]*",MODE="0666"
KERNEL=="ttyACM[0-9]*",MODE="0666"