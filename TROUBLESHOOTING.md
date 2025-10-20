# FTC 7215 Turning Gears Troubleshooting Guide

## Control Hub Connectivity
Sometimes the PC and Control Hub can lose connectivity. Try each step in order; stop if connectivity is restored. Otherwise, proceed to the next step.

### Wired ADB (USB)
1. Unplug the USB cable from the computer, then plug it back in.
2. Open the Rev Hardware Client.
3. Open Windows Terminal and run the following commands:
```
cd $HOME\AppData\Local\Android\sdk\platform-tools
./adb kill-server
./adb devices
```
4. Restart the Control Hub.
5. Restart the PC.

### Wireless ADB (Wi-Fi)
1. Ensure you're connected to the Control Hub Wi-fi network (FTC-7215-RC-0x).
2. Open the Rev Hardware Client.
3. Open Windows Terminal and run the following commands:
```
cd $HOME\AppData\Local\Android\sdk\platform-tools
./adb kill-server
./adb connect 192.168.43.1:5555
./adb devices
```
4. Restart the Control Hub.
5. Restart the PC.