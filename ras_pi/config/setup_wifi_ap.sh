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

# Stop conflicting services that prevent hostapd from working
echo "Stopping conflicting services..."

# Stop wpa_supplicant (conflicts with hostapd on same interface)
if pgrep wpa_supplicant >/dev/null 2>&1; then
    echo "Stopping wpa_supplicant..."
    sudo pkill wpa_supplicant || true
    sudo systemctl stop wpa_supplicant || true
    sudo systemctl disable wpa_supplicant || true
fi

# Stop NetworkManager on wlan0
if systemctl is-active NetworkManager >/dev/null 2>&1; then
    echo "Disconnecting NetworkManager from wlan0..."
    sudo nmcli device disconnect wlan0 || true
    sudo nmcli device set wlan0 managed no || true
fi

# Fix potential DNS port conflicts
echo "Resolving DNS port conflicts..."
# Stop systemd-resolved if it's running and conflicting
if systemctl is-active systemd-resolved >/dev/null 2>&1; then
    echo "Stopping systemd-resolved to free port 53..."
    sudo systemctl stop systemd-resolved
    sudo systemctl disable systemd-resolved
    # Fix DNS resolution
    sudo rm -f /etc/resolv.conf
    echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf
    echo "nameserver 8.8.4.4" | sudo tee -a /etc/resolv.conf
fi

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
# Disable DNS to avoid port 53 conflicts, only provide DHCP
port=0
dhcp-range=192.168.4.10,192.168.4.50,255.255.255.0,24h
dhcp-option=3,192.168.4.1
dhcp-option=6,8.8.8.8
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

# Fix any NetworkManager conflicts
echo "Preventing NetworkManager conflicts..."
if systemctl is-enabled NetworkManager >/dev/null 2>&1; then
    echo "Configuring NetworkManager to ignore wlan0..."
    # Create NetworkManager config to ignore wlan0
    sudo mkdir -p /etc/NetworkManager/conf.d
    sudo bash -c 'cat > /etc/NetworkManager/conf.d/99-unmanaged-devices.conf <<EOF
[keyfile]
unmanaged-devices=interface-name:wlan0
EOF'
    # Also add to main config as backup
    if ! grep -q "unmanaged-devices" /etc/NetworkManager/NetworkManager.conf; then
        sudo bash -c 'cat >> /etc/NetworkManager/NetworkManager.conf <<EOF

[keyfile]
unmanaged-devices=interface-name:wlan0
EOF'
    fi
    # Restart NetworkManager to apply changes
    sudo systemctl restart NetworkManager || true
fi

# Ensure WiFi radio is unblocked
echo "Unblocking WiFi radio..."
sudo rfkill unblock wifi || true

# Reset wlan0 interface for hostapd control
echo "Preparing wlan0 interface for AP mode..."
sudo ip link set wlan0 down || true
sudo iw dev wlan0 set type __ap || true
sudo ip link set wlan0 up || true

# Final restart sequence
echo "Starting services in correct order..."
sudo systemctl restart hostapd
sudo systemctl restart dnsmasq

# Verify services started correctly
echo "Verifying service status..."
if systemctl is-active hostapd >/dev/null 2>&1; then
    echo "✅ hostapd is running"
else
    echo "❌ hostapd failed to start"
fi

if systemctl is-active dnsmasq >/dev/null 2>&1; then
    echo "✅ dnsmasq is running"
else
    echo "❌ dnsmasq failed to start"
fi

echo "Setup complete. Access Point should be ready."
echo "SSID: MultiRobot_Network"
echo "Password: multirobot2024"
echo "AP IP: 192.168.4.1"
echo ""
echo "If devices still can't connect, run the diagnostic script:"
echo "sudo ./diagnose_wifi_ap.sh"
