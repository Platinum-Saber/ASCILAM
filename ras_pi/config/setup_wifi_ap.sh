#!/bin/bash
# Script to set up WiFi Access Point on Raspberry Pi 4 (Ubuntu MATE 20.04)
# This script must be run as root (sudo)

set -e

# 1.2 Install hostapd and dnsmasq
echo "Installing hostapd and dnsmasq..."
sudo apt update
sudo apt install -y hostapd dnsmasq

# Stop services before configuration
echo "Stopping hostapd and dnsmasq services..."
sudo systemctl stop hostapd || true
sudo systemctl stop dnsmasq || true

# 1.2 Configure static IP for wlan0
echo "Configuring static IP for wlan0..."
sudo bash -c 'cat >> /etc/dhcpcd.conf <<EOF
interface wlan0
static ip_address=192.168.4.1/24
nohook wpa_supplicant
EOF'

# 1.3 Configure dnsmasq
echo "Configuring dnsmasq..."
sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig || true
sudo bash -c 'cat > /etc/dnsmasq.conf <<EOF
interface=wlan0
dhcp-range=192.168.4.10,192.168.4.50,255.255.255.0,24h
dhcp-option=3,192.168.4.1
dhcp-option=6,192.168.4.1
# Reserve IPs for robots
dhcp-host=robot1,192.168.4.10
dhcp-host=robot2,192.168.4.20
EOF'

# 1.4 Configure hostapd
echo "Configuring hostapd..."
sudo bash -c 'cat > /etc/hostapd/hostapd.conf <<EOF
interface=wlan0
driver=nl80211
ssid=MultiRobot_Network
hw_mode=g
channel=6
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=multirobot2024
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
EOF'

# 1.5 Configure hostapd daemon
echo "Setting hostapd default config..."
sudo sed -i 's|^#DAEMON_CONF=.*|DAEMON_CONF="/etc/hostapd/hostapd.conf"|' /etc/default/hostapd

# Enable IP forwarding
echo "Enabling IP forwarding..."
sudo sed -i 's|^#net.ipv4.ip_forward=1|net.ipv4.ip_forward=1|' /etc/sysctl.conf

# Configure iptables for NAT
echo "Configuring iptables for NAT..."
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sudo iptables -A FORWARD -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i wlan0 -o eth0 -j ACCEPT
sudo sh -c "iptables-save > /etc/iptables.ipv4.nat"

# Auto-restore iptables on boot
echo "Configuring iptables auto-restore..."
# Create /etc/rc.local if it does not exist
if [ ! -f /etc/rc.local ]; then
    echo -e '#!/bin/sh -e\nexit 0' | sudo tee /etc/rc.local
    sudo chmod +x /etc/rc.local
fi
if ! grep -q 'iptables-restore < /etc/iptables.ipv4.nat' /etc/rc.local; then
    sudo sed -i '/^exit 0/i iptables-restore < /etc/iptables.ipv4.nat' /etc/rc.local
fi

# Enable services
echo "Enabling hostapd and dnsmasq services..."
sudo systemctl enable hostapd
sudo systemctl enable dnsmasq

echo "Setup complete. Please reboot to apply all changes."
