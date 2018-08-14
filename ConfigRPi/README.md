# Configuration steps for Raspberry Pi setup.

This guide describes the setup guide for getting a Raspberry Pi 3 working for our robot. Beside OS install, this includes
network configuration and enabling serial ports. Example files are included in this folder for quicker setup.

## OS and ethernet config.

 - Download the latest image of [Raspbian](https://www.raspberrypi.org/downloads/raspbian/)
 (here without desktop, but you can take the desktop image if needed). Current version: 4.14 kernel, release date 2018-06-27.
 - Extract img file to SD card (this creates a partition called rootfs).
 - Enable ssh on startup by creating an empty ssh file in boot partition (not /boot folder).
 - Edit /etc/dhcpcd.conf to have a static fallback address. I will use 192.168.8.2 as the Beaglebone Black uses 192.168.7.2.
 This is done by adding the following lines to the file:
```
define static profile
    profile static_eth0
    static ip_address=192.168.8.2/24

interface eth0
    fallback static_eth0
```

 - Boot the RPi and connect to PC with ethernet.
 - Give a static IP address to your PC: sudo ip addr add 192.168.8.3/24 dev <pc_ethernet_adapter_name>
 - Connect to ssh to RPI (username pi, password raspberry) at 192.168.8.2.
 - Use raspi-config to enable UART, I2C and SPI ports. The corresponding files should appear in /dev/

## Additional package install
 - Connect RPi to the internet, through wifi: use raspi-config for that.
  sudo apt-get update
  sudo apt-get upgrade
 - Update RPi, and install hostapd, dnsmasq (wifi config) rng-tools (for random number / wifi authentification) and i2c utils.
  sudo apt-get install hostapd dnsmasq i2c-tools rng-tools
 - Install glib library.
sudo apt-get install libglib2.0-dev

## Wifi config

We will now configure the wifi adapter to broadcast a network to connect to the raspberry pi.
See: https://www.raspberrypi.org/documentation/configuration/wireless/access-point.md

 - Reset wpa_supplicant config file, if needed (to prevent autoconnect to another wifi network).
 - Use hostapd to create the network: create a file /etc/hostadp/hostapd.conf with the following content:

interface=wlan0
driver=nl80211
bridge=br0
ssid=<NetworkName>
channel=1
wmm_enabled=0
wpa=1
wpa_passphrase=<Password>
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
auth_algs=1

 - Give a static IP address to the raspberry pi: append to /etc/dhcpcd.conf
	interface wlan0
		static ip_address=192.168.6.2/24
		nohook wpa_supplicant

 - Use dnsmasq to run a dhcp server, giving adress to a PC trying to connect to the raspberry.
 Create a file /etc/dnsmasq.conf with the following content:

  interface=wlan0
	dhcp-range=192.168.6.3,192.168.6.20,255.255.255.0,24h


 - Run on startup: edit /etc/default/hostapd to add
	DAEMON_CONF="/etc/hostapd/hostapd.conf"

	sudo systemctl start hostapd
	sudo systemctl start dnsmasq

 - To connect back to another wifi, simply comment the config in /etc/dhcpcd.conf.

An image with this config was created (with default password, network RaspberryPi/password RaspberryPi) using the following command:
sudo dd bs=4M if=/dev/mmcblk0 | gzip > 20180813-Raspbian4-14.img.gz
To restore, use:
cat 20180813-Raspbian4-14.img.gz | gunzip | dd of=/dev/mmcblk0

MiAM-Raspberry
Current user password: MiAMpy
Current wifi password: WifiMiAM
