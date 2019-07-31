# GPS Sensor

## Hardware
NEO-6M aGPS Module

## Connection
- VCC -> 3V Power (17)
- RX -> BCM14 / TXD (8)
- TX -> BCM15 / RXD (10)
- GND -> Ground

## Setup
- Install GPS Modules `sudo apt-get install gpsd gpsd-clients python-gps minicom`

1. paste `dwc_otg.lpm_enable=0 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait quiet splash plymouth.ignore-serial-consoles` 
to `/boot/cmdline.txt`
2. append `/boot/config.txt` with
    ```
    dtparam=spi=on
    dtoverlay=pi3-disable-bt
    core_freq=250
    enable_uart=1
    force_turbo=1
    init_uart_baud=9600
    ```
3. Set 9600 Rate `stty -F /dev/ttyAMA0 9600`
4. Add AMA0 to Devices (`/etc/default/gpsd`) by adding `DEVICES="/dev/ttyAMA0"`
5. Restart Software
    ```
    sudo systemctl enable gpsd.socket
    sudo systemctl start gpsd.socket 
    ```
    
## View Data Collected
`sudo cgps -s` or 
`cat /dev/ttyAMA0`

## Python Module used
gps

