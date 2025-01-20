# Engineering Team Motor
![MotorAssembly.png](images%2FMotorAssembly.png)

## Motor Controller PCB 
![Schematic.png](images%2FSchematic.png)
![PCB.png](images%2FPCB.png) 
![PCB_Assembly.png](images%2FPCB_Assembly.png)

### Microcontroller
Microcontroller used is an Arduino RP2040
![Arduino_RP2040.png](images%2FArduino_RP2040.png)
https://au.mouser.com/ProductDetail/Arduino/ABX00052?qs=sGAEpiMZZMuqBwn8WqcFUipNgoezRlc4TyjmMe5QIz0GNOLAi4TKQQ%3D%3D

### Stepper Motor Board:
![DRV8824.png](images%2FDRV8824.png)
https://www.pololu.com/product/2133

### Stepper motor
3D Printer Stepper Motor Model: 42BYGH34-1304B
![Motor.png](images%2FMotor.png)

### ABZ Quadrature Encoder
600 PPR ABZ Quadrature Encoder model number C38S6G5-600Z 
![Encoder.png](images%2FEncoder.png)

## Add User to dialout and TTY
Add user to dialout and tty to use serial devices

1. Add your standard user to the group "dialout"
`sudo usermod -a -G dialout your-username`

2. Add your standard user to the group "tty"
`sudo usermod -a -G tty your-username`

3. Logout/Login

## Named USB Serial Devices

http://www.reactivated.net/writing_udev_rules.html

By default usb serial devices get automatically named /dev/ttyUSB0 /dev/ttyACM0 .etc. 
This naming depends on the order the devices are plugged in. 
To give the usb serial devices fixed names we need to create some UDev rules.
Udev rules look at the attributes of the devices on your computer. If the attributes match a set of filters the device is given a fixed name.

Change to the udev rules directory.

`cd /etc/udev/rules.d`

Create a file to hold our new rules for project. The 10- causes our new rule to run before other rules.

`sudo touch 10-motor.rules`

Edit the new file and add the following lines. (SYMLINK is a string identifier can be anything)
`sudo gedit 10-motor.rules`

```
SUBSYSTEM=="tty", ATTRS{idProduct}=="005e", ATTRS{idVendor}=="2341", SYMLINK+="arduino_rp2040"
```

For reference, the idProject and idVendor come from inspecting the serial devices with.

`udevadm info --name=/dev/ttyACM0 --attribute-walk`

Which outputs a heap of stuff like this.

```Udevadm info starts with the device specified by the devpath and then
walks up the chain of parent devices. It prints for every device
found, all possible attributes in the udev rules key format.
A rule to match, can be composed by the attributes of the device
and the attributes from one single parent device.

  looking at device '/devices/pci0000:00/0000:00:14.0/usb1/1-10/1-10:1.0/tty/ttyACM0':
    KERNEL=="ttyACM0"
    SUBSYSTEM=="tty"
    DRIVER==""

  looking at parent device '/devices/pci0000:00/0000:00:14.0/usb1/1-10/1-10:1.0':
    KERNELS=="1-10:1.0"
    SUBSYSTEMS=="usb"
    DRIVERS=="cdc_acm"
    ATTRS{bmCapabilities}=="6"
    ATTRS{iad_bFunctionSubClass}=="02"
    ATTRS{bInterfaceNumber}=="00"
    ATTRS{bInterfaceClass}=="02"
    ATTRS{bNumEndpoints}=="01"
    ATTRS{supports_autosuspend}=="1"
    ATTRS{iad_bInterfaceCount}=="02"
    ATTRS{iad_bFirstInterface}=="00"
    ATTRS{bAlternateSetting}==" 0"
    ATTRS{bInterfaceSubClass}=="02"
    ATTRS{iad_bFunctionClass}=="02"
    ATTRS{iad_bFunctionProtocol}=="00"
    ATTRS{authorized}=="1"
    ATTRS{bInterfaceProtocol}=="01"

  looking at parent device '/devices/pci0000:00/0000:00:14.0/usb1/1-10':
    KERNELS=="1-10"
    SUBSYSTEMS=="usb"
    DRIVERS=="usb"
    ATTRS{quirks}=="0x0"
    ATTRS{version}==" 2.00"
    ATTRS{removable}=="removable"
    ATTRS{devpath}=="10"
    ATTRS{bDeviceSubClass}=="02"
    ATTRS{idVendor}=="2341"
    ATTRS{bmAttributes}=="c0"
    ATTRS{bConfigurationValue}=="1"
    ATTRS{bDeviceClass}=="ef"
    ATTRS{avoid_reset_quirk}=="0"
    ATTRS{speed}=="12"
    ATTRS{busnum}=="1"
    ATTRS{idProduct}=="005e"
    ATTRS{serial}=="416523501C7861C8"
    ATTRS{product}=="Nano RP2040 Connect"
    ATTRS{configuration}==""
    ATTRS{devnum}=="12"
    ATTRS{bNumInterfaces}==" 2"
    ATTRS{bMaxPacketSize0}=="64"
    ATTRS{bMaxPower}=="500mA"
    ATTRS{bDeviceProtocol}=="01"
    ATTRS{manufacturer}=="Arduino"
    ATTRS{maxchild}=="0"
    ATTRS{authorized}=="1"
    ATTRS{rx_lanes}=="1"
    ATTRS{urbnum}=="13"
    ATTRS{ltm_capable}=="no"
    ATTRS{bNumConfigurations}=="1"
    ATTRS{tx_lanes}=="1"
    ATTRS{bcdDevice}=="0101"

  looking at parent device '/devices/pci0000:00/0000:00:14.0/usb1':
    KERNELS=="usb1"
    SUBSYSTEMS=="usb"
    DRIVERS=="usb"
    ATTRS{tx_lanes}=="1"
    ATTRS{product}=="xHCI Host Controller"
    ATTRS{maxchild}=="16"
    ATTRS{bDeviceClass}=="09"
    ATTRS{bDeviceProtocol}=="01"
    ATTRS{removable}=="unknown"
    ATTRS{urbnum}=="178"
    ATTRS{speed}=="480"
    ATTRS{bDeviceSubClass}=="00"
    ATTRS{bConfigurationValue}=="1"
    ATTRS{busnum}=="1"
    ATTRS{idVendor}=="1d6b"
    ATTRS{authorized}=="1"
    ATTRS{bNumConfigurations}=="1"
    ATTRS{bMaxPower}=="0mA"
    ATTRS{manufacturer}=="Linux 5.15.0-130-generic xhci-hcd"
    ATTRS{devpath}=="0"
    ATTRS{bMaxPacketSize0}=="64"
    ATTRS{quirks}=="0x0"
    ATTRS{ltm_capable}=="no"
    ATTRS{serial}=="0000:00:14.0"
    ATTRS{avoid_reset_quirk}=="0"
    ATTRS{bmAttributes}=="e0"
    ATTRS{bNumInterfaces}==" 1"
    ATTRS{devnum}=="1"
    ATTRS{version}==" 2.00"
    ATTRS{configuration}==""
    ATTRS{bcdDevice}=="0515"
    ATTRS{rx_lanes}=="1"
    ATTRS{interface_authorized_default}=="1"
    ATTRS{authorized_default}=="1"
    ATTRS{idProduct}=="0002"

  looking at parent device '/devices/pci0000:00/0000:00:14.0':
    KERNELS=="0000:00:14.0"
    SUBSYSTEMS=="pci"
    DRIVERS=="xhci_hcd"
    ATTRS{revision}=="0x10"
    ATTRS{class}=="0x0c0330"
    ATTRS{index}=="3"
    ATTRS{dbc}=="disabled"
    ATTRS{label}=="Onboard - Other"
    ATTRS{local_cpus}=="ff"
    ATTRS{device}=="0xa36d"
    ATTRS{irq}=="127"
    ATTRS{d3cold_allowed}=="1"
    ATTRS{local_cpulist}=="0-7"
    ATTRS{driver_override}=="(null)"
    ATTRS{msi_bus}=="1"
    ATTRS{vendor}=="0x8086"
    ATTRS{numa_node}=="-1"
    ATTRS{ari_enabled}=="0"
    ATTRS{dma_mask_bits}=="64"
    ATTRS{subsystem_vendor}=="0x1458"
    ATTRS{power_state}=="D0"
    ATTRS{broken_parity_status}=="0"
    ATTRS{consistent_dma_mask_bits}=="64"
    ATTRS{enable}=="1"
    ATTRS{subsystem_device}=="0x5007"

  looking at parent device '/devices/pci0000:00':
    KERNELS=="pci0000:00"
    SUBSYSTEMS==""
    DRIVERS==""
    ATTRS{waiting_for_supplier}=="0"
```

Reload udev rules and trigger udev events

`sudo udevadm control --reload-rules && sudo udevadm trigger`

Switch to /dev directory
`cd /dev`

Check /dev shows SYMLINK 
`ls`

![dev.png](images%2Fdev.png)

## Python Interface
Basic job example:

```
  from pathlib import Path
  from motor import Motor
  
  project_dir = Path(__file__).resolve().parents[1]
  header_file = project_dir / 'arduino/engineering-team-motor/definitions.h'

  motor = Motor(definitions_filepath=header_file, serial_port='/dev/arduino_rp2040')
  motor.start_threads()
  motor.send_enable_motor()

  try:
    while True:
      if motor.is_ready_for_job():
        time.sleep(1)
        motor.send_wake_motor()
        motor.send_motor_rotations_at_set_rpm(number_or_rotations=1,
                                              rpm=random.random() * 10,
                                              direction=random.choice([True, False]),
                                              job_id=1)
   
      else:
        time.sleep(0.5)
        print(f"{motor.get_rotor_position()=:.3f} radians")
        print(motor.status_message_dict)
    
  except KeyboardInterrupt:
    motor.stop()
```

### Job Types
  send_motor_rotations_at_set_rpm\
  send_motor_pulses_at_set_rpm\
  send_motor_rotations\
  goto_rotor_position_radians

