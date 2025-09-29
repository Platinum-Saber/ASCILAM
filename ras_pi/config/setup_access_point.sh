#!/bin/bash
# Fixed Safe WiFi Access Point Setup for Ubuntu MATE 20.04
# Preserves ethernet connectivity while creating WiFi AP
# Works with existing NetworkManager and systemd-networkd

# Remove strict error checking for network operations that may already exist
# set -e

echo "========================================="
echo "Ubuntu MATE Safe Access Point Setup (Fixed)"
echo "========================================="
echo "This script will preserve your ethernet connection"
echo "while setting up WiFi as an Access Point"
echo ""

# Function to print section headers
print_section() {
    echo ""
    echo "--- $1 ---"
}

# Function to check command success (modified to be more permissive)
check_result() {
    local exit_code=$?
    if [ $exit_code -eq 0 ]; then
        echo "✅ $1"
        return 0
    else
        echo "⚠️  $1 - Warning (exit code: $exit_code)"
        return 0  # Don't exit on warnings
    fi
}

# Function to backup files safely
backup_file() {
    local file="$1"
    if [ -f "$file" ] && [ ! -f "$file.backup-$(date +%Y%m%d)" ]; then
        sudo cp "$file" "$file.backup-$(date +%Y%m%d)"
        echo "✅ Backed up $file"
    fi
}

print_section "0. Pre-flight Checks"

# Verify ethernet connection is working
echo "Checking ethernet connectivity..."
if ping -c 1 192.168.100.1 >/dev/null 2>&1; then
    echo "✅ Ethernet connection to Windows PC verified"
else
    echo "⚠️  Warning: Cannot ping Windows PC (192.168.100.1)"
    echo "Continuing anyway..."
fi

# Check current network status
echo ""
echo "Current network configuration:"
ip addr show eth0 | grep "inet " || echo "No ethernet IP found"
ip addr show wlan0 | grep "inet " || echo "No WiFi IP"
echo ""

read -p "Continue with Access Point setup? (y/N): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Setup cancelled."
    exit 0
fi

print_section "1. Installing Required Packages"

# Check internet connectivity before attempting package operations
echo "Checking internet connectivity..."
if ping -c 1 -W 3 8.8.8.8 >/dev/null 2>&1; then
    echo "✅ Internet available - updating package lists"
    sudo apt update
    sudo apt install -y hostapd dnsmasq
    check_result "Package installation"
else
    echo "⚠️  No internet connection detected - skipping apt update"
    echo "Attempting to install packages with existing cache..."
    
    # Check if packages are already installed
    if dpkg -l | grep -q "hostapd" && dpkg -l | grep -q "dnsmasq"; then
        echo "✅ Required packages already installed"
    else
        echo "Attempting installation without update..."
        if sudo apt install -y hostapd dnsmasq; then
            echo "✅ Package installation successful"
        else
            echo "❌ Package installation failed - packages may need to be installed manually"
            echo "Please ensure hostapd and dnsmasq are installed before continuing"
            read -p "Continue anyway? (y/N): " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                echo "Setup cancelled."
                exit 1
            fi
        fi
    fi
fi

print_section "2. Backing Up Configurations"

# Create backups with timestamps
backup_file "/etc/dnsmasq.conf"
backup_file "/etc/hostapd/hostapd.conf"

print_section "3. Configuring NetworkManager for Dual Interface"

# Configure NetworkManager to ignore wlan0 for AP mode
echo "Configuring NetworkManager to allow AP mode..."
sudo mkdir -p /etc/NetworkManager/conf.d

# Create specific config for AP mode (doesn't disable NetworkManager)
sudo tee /etc/NetworkManager/conf.d/99-wifi-ap.conf > /dev/null <<EOF
[keyfile]
unmanaged-devices=interface-name:wlan0

[device]
wifi.scan-rand-mac-address=no
EOF

check_result "NetworkManager AP configuration"

print_section "4. Configuring hostapd"

# Create hostapd configuration
sudo tee /etc/hostapd/hostapd.conf > /dev/null <<EOF
# Basic interface configuration
interface=wlan0
driver=nl80211

# Network identification
ssid=MultiRobot_Network
hw_mode=g
channel=7
ieee80211n=1

# Security settings
wpa=2
wpa_passphrase=multirobot2024
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP

# Access Point settings
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wmm_enabled=0

# Country code (adjust for your location)
country_code=US
EOF

# Configure hostapd daemon
if [ -f /etc/default/hostapd ]; then
    sudo sed -i 's|^#DAEMON_CONF=.*|DAEMON_CONF="/etc/hostapd/hostapd.conf"|' /etc/default/hostapd
fi

check_result "hostapd configuration"

print_section "5. Configuring dnsmasq for DHCP"

# Stop dnsmasq first to avoid conflicts
sudo systemctl stop dnsmasq 2>/dev/null || true

# Create focused dnsmasq config that won't interfere with main networking
sudo tee /etc/dnsmasq.conf > /dev/null <<EOF
# Bind only to wlan0 interface
interface=wlan0
bind-interfaces

# Disable DNS server (avoid conflicts with systemd-resolved)
port=0

# DHCP configuration for WiFi clients
dhcp-range=192.168.4.10,192.168.4.100,255.255.255.0,24h

# Network options
dhcp-option=3,192.168.4.1     # Gateway
dhcp-option=6,8.8.8.8,8.8.4.4 # DNS servers

# Reserved IPs for robots
dhcp-host=robot1,192.168.4.10
dhcp-host=robot2,192.168.4.20

# DHCP authoritative mode for this interface only
dhcp-authoritative

# Logging
log-dhcp
log-facility=/var/log/dnsmasq.log
EOF

check_result "dnsmasq configuration"

print_section "6. Setting up WiFi Interface with Static IP"

# Create systemd-networkd configuration for wlan0
sudo tee /etc/systemd/network/99-wlan0-ap.network > /dev/null <<EOF
[Match]
Name=wlan0

[Network]
Address=192.168.4.1/24
IPMasquerade=yes
IPForward=yes
DHCPServer=no

[Route]
Destination=192.168.4.0/24
EOF

check_result "WiFi interface configuration"

print_section "7. Configuring IP Forwarding and NAT"

# Enable IP forwarding (for internet sharing via ethernet)
echo "Enabling IP forwarding..."
if ! grep -q "net.ipv4.ip_forward=1" /etc/sysctl.conf; then
    echo "net.ipv4.ip_forward=1" | sudo tee -a /etc/sysctl.conf
fi
sudo sysctl -w net.ipv4.ip_forward=1

# Configure iptables for NAT (sharing ethernet internet to WiFi clients)
echo "Setting up NAT rules..."

# Clear any existing rules for our chains (don't fail if they don't exist)
sudo iptables -t nat -D POSTROUTING -s 192.168.4.0/24 -o eth0 -j MASQUERADE 2>/dev/null || true
sudo iptables -D FORWARD -i wlan0 -o eth0 -j ACCEPT 2>/dev/null || true
sudo iptables -D FORWARD -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true

# Add new rules
sudo iptables -t nat -A POSTROUTING -s 192.168.4.0/24 -o eth0 -j MASQUERADE
sudo iptables -A FORWARD -i wlan0 -o eth0 -j ACCEPT
sudo iptables -A FORWARD -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT

# Save iptables rules
sudo mkdir -p /etc/iptables
sudo sh -c "iptables-save > /etc/iptables/rules.v4"

# Create restore script for boot
sudo tee /etc/systemd/system/iptables-restore.service > /dev/null <<EOF
[Unit]
Description=Restore iptables rules
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/iptables-restore /etc/iptables/rules.v4
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable iptables-restore.service

check_result "NAT configuration"

print_section "8. Preparing WiFi Interface"

# Ensure WiFi is unblocked
sudo rfkill unblock wifi

# Stop any existing WiFi connections managed by NetworkManager
echo "Disconnecting WiFi from NetworkManager..."
sudo nmcli device disconnect wlan0 2>/dev/null || true

# Carefully restart NetworkManager to apply new config
echo "Restarting NetworkManager..."
sudo systemctl restart NetworkManager
sleep 3

# Verify ethernet is still working
if ping -c 1 192.168.100.1 >/dev/null 2>&1; then
    echo "✅ Ethernet connection preserved"
else
    echo "⚠️  Warning: Ethernet connectivity may be affected"
fi

# Prepare wlan0 interface - handle existing configurations gracefully
echo "Preparing wlan0 interface..."

# Bring interface down and flush existing addresses
sudo ip link set wlan0 down 2>/dev/null || true
sudo ip addr flush dev wlan0 2>/dev/null || true

# Set interface type
sudo iw dev wlan0 set type __ap 2>/dev/null || sudo iw dev wlan0 set type managed 2>/dev/null || true

# Bring interface up
sudo ip link set wlan0 up

# Add IP address only if it doesn't exist
if ! ip addr show wlan0 | grep -q "192.168.4.1/24"; then
    sudo ip addr add 192.168.4.1/24 dev wlan0
    echo "✅ Added IP address to wlan0"
else
    echo "✅ IP address already exists on wlan0"
fi

check_result "WiFi interface preparation"

print_section "9. Starting Access Point Services"

# Enable services but don't start them yet
sudo systemctl enable hostapd
sudo systemctl enable dnsmasq

# Stop services first to ensure clean start
sudo systemctl stop hostapd 2>/dev/null || true
sudo systemctl stop dnsmasq 2>/dev/null || true

# Wait a moment for services to fully stop
sleep 2

# Start services in correct order
echo "Starting hostapd..."
if sudo systemctl start hostapd; then
    echo "✅ hostapd started successfully"
else
    echo "⚠️  hostapd failed to start - checking logs"
    sudo journalctl -u hostapd --no-pager -n 5
fi

sleep 3

echo "Starting dnsmasq..."
if sudo systemctl start dnsmasq; then
    echo "✅ dnsmasq started successfully"
else
    echo "⚠️  dnsmasq failed to start - checking logs"
    sudo journalctl -u dnsmasq --no-pager -n 5
fi

sleep 2

check_result "Access Point services started"

print_section "10. Verification and Status"

echo "Checking service status..."

# Check hostapd
if systemctl is-active hostapd >/dev/null 2>&1; then
    echo "✅ hostapd is running"
else
    echo "❌ hostapd failed to start"
    echo "Recent hostapd logs:"
    sudo journalctl -u hostapd --no-pager -n 5
fi

# Check dnsmasq
if systemctl is-active dnsmasq >/dev/null 2>&1; then
    echo "✅ dnsmasq is running"
else
    echo "❌ dnsmasq failed to start"
    echo "Recent dnsmasq logs:"
    sudo journalctl -u dnsmasq --no-pager -n 5
fi

echo ""
echo "Network interface status:"
echo "Ethernet (eth0):"
ip addr show eth0 | grep "inet " || echo "  No IP assigned"
echo "WiFi AP (wlan0):"
ip addr show wlan0 | grep "inet " || echo "  No IP assigned"

echo ""
echo "Testing connectivity:"
ping -c 1 192.168.100.1 >/dev/null 2>&1 && echo "✅ Ethernet to Windows: Working" || echo "❌ Ethernet to Windows: Failed"
ping -c 1 8.8.8.8 >/dev/null 2>&1 && echo "✅ Internet access: Working" || echo "❌ Internet access: Failed"

echo ""
echo "WiFi interface mode:"
iw dev wlan0 info | grep type || echo "Could not determine WiFi mode"

print_section "11. Setup Complete"

echo ""
echo "🎉 Safe Access Point setup completed!"
echo ""
echo "Network Configuration:"
echo "  📡 WiFi Access Point:"
echo "    SSID: MultiRobot_Network"
echo "    Password: multirobot2024"
echo "    AP IP: 192.168.4.1"
echo "    Client Range: 192.168.4.10-192.168.4.100"
echo ""
echo "  🔌 Ethernet (preserved):"
echo "    IP: 192.168.100.2"
echo "    Gateway: Windows PC (192.168.100.1)"
echo ""
echo "  🤖 Reserved Robot IPs:"
echo "    robot1: 192.168.4.10"
echo "    robot2: 192.168.4.20"
echo ""

# Create monitoring script with better error handling
tee /home/$(logname)/monitor_ap.sh > /dev/null <<'EOF'
#!/bin/bash
echo "=== Access Point Status Monitor ==="
echo ""

echo "🔧 Services:"
systemctl is-active hostapd >/dev/null && echo "✅ hostapd: running" || echo "❌ hostapd: stopped"
systemctl is-active dnsmasq >/dev/null && echo "✅ dnsmasq: running" || echo "❌ dnsmasq: stopped"
systemctl is-active NetworkManager >/dev/null && echo "✅ NetworkManager: running" || echo "❌ NetworkManager: stopped"

echo ""
echo "🌐 Network Interfaces:"
echo "Ethernet:"
ip addr show eth0 | grep "inet " | awk '{print "  " $2}' || echo "  No IP"
echo "WiFi AP:"
ip addr show wlan0 | grep "inet " | awk '{print "  " $2}' || echo "  No IP"

echo ""
echo "🔗 Connectivity:"
ping -c 1 192.168.100.1 >/dev/null 2>&1 && echo "✅ Windows PC: reachable" || echo "❌ Windows PC: unreachable"
ping -c 1 8.8.8.8 >/dev/null 2>&1 && echo "✅ Internet: reachable" || echo "❌ Internet: unreachable"

echo ""
echo "📱 Connected WiFi Clients:"
client_count=$(sudo iw dev wlan0 station dump 2>/dev/null | grep "Station" | wc -l)
echo "  Count: $client_count"
sudo iw dev wlan0 station dump 2>/dev/null | grep "Station" | awk '{print "  " $2}' || echo "  None"

echo ""
echo "📋 DHCP Leases:"
if [ -f /var/lib/dhcp/dhcpd.leases ]; then
    tail -20 /var/lib/dhcp/dhcpd.leases | grep "lease\|client-hostname\|hardware ethernet" | tail -10
elif [ -f /var/lib/misc/dnsmasq.leases ]; then
    cat /var/lib/misc/dnsmasq.leases | awk '{print "  " $4 " -> " $3}' || echo "  No leases"
else
    echo "  No lease file found"
fi

echo ""
echo "📊 Recent DHCP Activity:"
if [ -f /var/log/dnsmasq.log ]; then
    sudo tail -5 /var/log/dnsmasq.log 2>/dev/null || echo "  No recent activity"
else
    echo "  No log file found"
fi
EOF

chmod +x /home/$(logname)/monitor_ap.sh

echo "Monitoring commands:"
echo "  📊 Status monitor: ./monitor_ap.sh"
echo "  📝 Live hostapd logs: sudo journalctl -u hostapd -f"
echo "  📝 Live dnsmasq logs: sudo journalctl -u dnsmasq -f"
echo "  📝 Connected devices: sudo iw dev wlan0 station dump"
echo ""
echo "To stop Access Point (preserving ethernet):"
echo "  sudo systemctl stop hostapd dnsmasq"
echo ""
echo "To restart services if needed:"
echo "  sudo systemctl restart hostapd dnsmasq"
echo ""
echo "⚠️  Note: Your ethernet connection to Windows should remain active"
echo "   Test it with: ping 192.168.100.1"
echo ""

# Final status check
if systemctl is-active hostapd >/dev/null 2>&1 && systemctl is-active dnsmasq >/dev/null 2>&1; then
    echo "✅ All services are running - Access Point should be operational!"
else
    echo "⚠️  Some services may not be running - check with ./monitor_ap.sh"
fi