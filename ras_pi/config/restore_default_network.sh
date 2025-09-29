#!/bin/bash
# Script to restore default network configuration on Raspberry Pi 4 (Ubuntu MATE 20.04)
# This undoes the WiFi Access Point setup and restores normal WiFi client mode
# Updated for Ubuntu MATE environment with NetworkManager and systemd-networkd

set -e

echo "========================================="
echo "Ubuntu MATE Network Configuration Restore"
echo "========================================="

# Function to print section headers
print_section() {
    echo ""
    echo "--- $1 ---"
}

# Function to check command success
check_result() {
    if [ $? -eq 0 ]; then
        echo "✅ $1"
    else
        echo "⚠️  $1 - Some issues (continuing...)"
    fi
}

# Function to detect network management system
detect_network_manager() {
    if systemctl is-active NetworkManager >/dev/null 2>&1; then
        echo "NetworkManager"
    elif systemctl is-active systemd-networkd >/dev/null 2>&1; then
        echo "systemd-networkd"
    elif systemctl is-active dhcpcd >/dev/null 2>&1; then
        echo "dhcpcd"
    else
        echo "unknown"
    fi
}

NETWORK_MANAGER=$(detect_network_manager)
echo "Detected network management system: $NETWORK_MANAGER"

print_section "1. Stopping Access Point Services"

# Stop AP services
sudo systemctl stop hostapd || true
sudo systemctl stop dnsmasq || true
sudo systemctl disable hostapd || true
sudo systemctl disable dnsmasq || true

# Mask hostapd to prevent auto-start
sudo systemctl mask hostapd || true

check_result "Stopped AP services"

print_section "2. Restoring Network Configuration"

# Handle different network management systems
case $NETWORK_MANAGER in
    "NetworkManager")
        echo "Using NetworkManager configuration..."
        # NetworkManager typically doesn't use dhcpcd.conf
        if [ -f /etc/dhcpcd.conf ]; then
            if [ -f /etc/dhcpcd.conf.backup ]; then
                echo "Restoring dhcpcd.conf from backup..."
                sudo cp /etc/dhcpcd.conf.backup /etc/dhcpcd.conf
            else
                echo "Removing AP configuration from dhcpcd.conf..."
                sudo sed -i '/# Static IP configuration for Access Point/,/denyinterfaces wlan0/d' /etc/dhcpcd.conf
                sudo sed -i '/^interface wlan0/,/^$/d' /etc/dhcpcd.conf
            fi
        fi
        check_result "Cleaned dhcpcd configuration"
        ;;
    "dhcpcd")
        echo "Using dhcpcd configuration..."
        if [ -f /etc/dhcpcd.conf.backup ]; then
            echo "Restoring dhcpcd.conf from backup..."
            sudo cp /etc/dhcpcd.conf.backup /etc/dhcpcd.conf
        else
            echo "Removing AP configuration from dhcpcd.conf..."
            sudo sed -i '/# Static IP configuration for Access Point/,/denyinterfaces wlan0/d' /etc/dhcpcd.conf
            sudo sed -i '/^interface wlan0/,/^$/d' /etc/dhcpcd.conf
        fi
        check_result "Restored dhcpcd configuration"
        ;;
    "systemd-networkd")
        echo "Using systemd-networkd configuration..."
        # Remove any AP-specific network files
        sudo rm -f /etc/systemd/network/*wlan0* || true
        sudo rm -f /etc/systemd/network/*ap* || true
        check_result "Cleaned systemd-networkd configuration"
        ;;
esac

print_section "3. Restoring dnsmasq Configuration"

# Restore original dnsmasq.conf if backup exists
if [ -f /etc/dnsmasq.conf.backup ]; then
    echo "Restoring dnsmasq.conf from backup..."
    sudo cp /etc/dnsmasq.conf.backup /etc/dnsmasq.conf
    check_result "Restored dnsmasq.conf from backup"
else
    echo "No dnsmasq backup found. Using default configuration..."
    sudo tee /etc/dnsmasq.conf > /dev/null <<'EOF'
# Configuration file for dnsmasq.
# See dnsmasq(8) for details of configuration options.
EOF
    check_result "Reset dnsmasq.conf to default"
fi

print_section "4. Removing hostapd Configuration"

# Remove hostapd configuration
if [ -f /etc/hostapd/hostapd.conf ]; then
    sudo rm /etc/hostapd/hostapd.conf
    check_result "Removed hostapd configuration"
fi

# Reset hostapd default config
if [ -f /etc/default/hostapd ]; then
    sudo sed -i 's|^DAEMON_CONF=.*|#DAEMON_CONF=""|' /etc/default/hostapd
fi

print_section "5. Restoring NetworkManager Configuration"

# Remove NetworkManager unmanaged devices configuration
if [ -f /etc/NetworkManager/conf.d/99-unmanaged-devices.conf ]; then
    sudo rm /etc/NetworkManager/conf.d/99-unmanaged-devices.conf
    check_result "Removed NetworkManager unmanaged devices config"
fi

# Remove unmanaged-devices from main NetworkManager.conf
if [ -f /etc/NetworkManager/NetworkManager.conf ]; then
    sudo sed -i '/unmanaged-devices=interface-name:wlan0/d' /etc/NetworkManager/NetworkManager.conf
    sudo sed -i '/^\[keyfile\]$/N;/^\[keyfile\]\n$/d' /etc/NetworkManager/NetworkManager.conf
fi

check_result "Restored NetworkManager configuration"

print_section "6. Restoring systemd-resolved"

# Re-enable systemd-resolved if it was disabled
if ! systemctl is-enabled systemd-resolved >/dev/null 2>&1; then
    echo "Re-enabling systemd-resolved..."
    sudo systemctl enable systemd-resolved
    sudo systemctl start systemd-resolved
    check_result "Restored systemd-resolved"
fi

print_section "7. Restoring DNS Configuration"

# Restore DNS configuration
sudo chattr -i /etc/resolv.conf 2>/dev/null || true
sudo rm -f /etc/resolv.conf

# Create symlink to systemd-resolved
sudo ln -sf /run/systemd/resolve/stub-resolv.conf /etc/resolv.conf
check_result "Restored DNS configuration"

print_section "8. Removing IP Forwarding"

# Disable IP forwarding
sudo sed -i 's|^net.ipv4.ip_forward=1|#net.ipv4.ip_forward=1|' /etc/sysctl.conf
sudo sysctl -p

check_result "Disabled IP forwarding"

print_section "9. Removing iptables Rules"

# Flush iptables NAT rules
sudo iptables -t nat -F POSTROUTING 2>/dev/null || true
sudo iptables -F FORWARD 2>/dev/null || true

# Remove saved iptables rules
sudo rm -f /etc/iptables.ipv4.nat
sudo rm -f /etc/iptables/rules.v4

# Remove from rc.local if it exists
if [ -f /etc/rc.local ]; then
    sudo sed -i '/iptables-restore < \/etc\/iptables.ipv4.nat/d' /etc/rc.local
fi

check_result "Removed iptables NAT rules"

print_section "10. Restoring WiFi Interface"

# Bring down wlan0
sudo ip link set wlan0 down

# Flush any static IP
sudo ip addr flush dev wlan0

# Set interface back to managed (client) mode
sudo iw dev wlan0 set type managed

# Bring interface back up
sudo ip link set wlan0 up

check_result "Reset wlan0 to managed mode"

print_section "11. Re-enabling WiFi Services"

# Re-enable wpa_supplicant
sudo systemctl unmask wpa_supplicant 2>/dev/null || true
sudo systemctl enable wpa_supplicant 2>/dev/null || true

# Restart appropriate network services based on detected system
case $NETWORK_MANAGER in
    "NetworkManager")
        echo "Restarting NetworkManager and systemd-networkd..."
        sudo systemctl restart NetworkManager
        sudo systemctl restart systemd-networkd
        # Ensure wlan0 is managed by NetworkManager
        sleep 2
        sudo nmcli device set wlan0 managed yes 2>/dev/null || true
        check_result "Restored NetworkManager WiFi services"
        ;;
    "dhcpcd")
        echo "Restarting dhcpcd..."
        if systemctl is-active dhcpcd >/dev/null 2>&1; then
            sudo systemctl restart dhcpcd
        else
            echo "dhcpcd not found, attempting to install and start..."
            sudo apt update && sudo apt install -y dhcpcd5
            sudo systemctl enable dhcpcd
            sudo systemctl start dhcpcd
        fi
        check_result "Restored dhcpcd WiFi services"
        ;;
    "systemd-networkd")
        echo "Restarting systemd-networkd..."
        sudo systemctl restart systemd-networkd
        check_result "Restored systemd-networkd WiFi services"
        ;;
    *)
        echo "Unknown network manager, trying NetworkManager..."
        sudo systemctl restart NetworkManager || true
        sudo systemctl restart systemd-networkd || true
        check_result "Attempted to restore network services"
        ;;
esac

print_section "12. Cleaning Up Additional Files"

# Remove test script if it exists
if [ -f /home/$(logname)/test_ap.sh ]; then
    rm /home/$(logname)/test_ap.sh
    check_result "Removed test script"
fi

# Remove any backup files older than original
find /etc -name "*.orig" -type f 2>/dev/null | while read file; do
    echo "Found backup: $file"
done

print_section "13. Verification"

echo "Checking interface status..."
ip addr show wlan0 | grep -E "(inet|state)" || echo "No IP assigned (normal for client mode)"

echo ""
echo "Checking WiFi mode..."
iw dev wlan0 info | grep type

echo ""
echo "Checking services..."
systemctl is-active NetworkManager >/dev/null 2>&1 && echo "✅ NetworkManager active" || echo "❌ NetworkManager inactive"
systemctl is-active systemd-networkd >/dev/null 2>&1 && echo "✅ systemd-networkd active" || echo "❌ systemd-networkd inactive"
systemctl is-active dhcpcd >/dev/null 2>&1 && echo "✅ dhcpcd active" || echo "❌ dhcpcd inactive/not installed"
systemctl is-active hostapd >/dev/null 2>&1 && echo "⚠️  hostapd still active" || echo "✅ hostapd stopped"
systemctl is-active dnsmasq >/dev/null 2>&1 && echo "⚠️  dnsmasq still active" || echo "✅ dnsmasq stopped"

echo ""
echo "NetworkManager device status:"
nmcli device status 2>/dev/null || echo "nmcli not available"

print_section "14. Restore Complete!"

echo ""
echo "🎉 Network configuration has been restored to default!"
echo ""
echo "Changes made:"
echo "  ✅ Stopped and disabled hostapd and dnsmasq"
echo "  ✅ Restored network configuration for $NETWORK_MANAGER"
echo "  ✅ Removed NetworkManager exclusions"
echo "  ✅ Re-enabled systemd-resolved"
echo "  ✅ Restored DNS configuration"
echo "  ✅ Removed IP forwarding and NAT rules"
echo "  ✅ Reset wlan0 to managed (client) mode"
echo "  ✅ Re-enabled standard WiFi services"
echo ""
echo "Your Ubuntu MATE system can now connect to WiFi networks normally."
echo ""

if command -v nmcli >/dev/null 2>&1; then
    echo "To connect to WiFi using NetworkManager:"
    echo "  1. Use the GUI network manager, or"
    echo "  2. Use command: sudo nmcli dev wifi connect \"SSID\" password \"PASSWORD\""
    echo ""
    echo "To scan for networks:"
    echo "  nmcli dev wifi list"
else
    echo "To connect to WiFi:"
    echo "  1. Use the GUI network manager, or"
    echo "  2. Configure /etc/wpa_supplicant/wpa_supplicant.conf manually"
fi

echo ""
echo "If you encounter issues:"
echo "  - Reboot the system: sudo reboot"
echo "  - Check WiFi status: nmcli device status"
echo "  - Check for hardware blocks: sudo rfkill list"
echo "  - Run the status script: ./ubuntu_network_status.sh"
echo ""

# Create Ubuntu MATE specific status check script
tee /home/$(logname)/ubuntu_network_status.sh > /dev/null <<'EOF'
#!/bin/bash
echo "=== Ubuntu MATE Network Status ==="
echo ""

echo "Network Management System:"
if systemctl is-active NetworkManager >/dev/null 2>&1; then
    echo "✅ NetworkManager: $(systemctl is-active NetworkManager)"
else
    echo "❌ NetworkManager: inactive"
fi

if systemctl is-active systemd-networkd >/dev/null 2>&1; then
    echo "✅ systemd-networkd: $(systemctl is-active systemd-networkd)"
else
    echo "❌ systemd-networkd: inactive"
fi

if systemctl is-active dhcpcd >/dev/null 2>&1; then
    echo "✅ dhcpcd: $(systemctl is-active dhcpcd)"
else
    echo "❌ dhcpcd: inactive/not installed"
fi

echo ""
echo "Network Interfaces:"
ip addr show | grep -E "(eth0|wlan0)" -A 2

echo ""
echo "WiFi Interface Mode:"
iw dev wlan0 info 2>/dev/null | grep type || echo "WiFi interface not found"

echo ""
echo "Connectivity Tests:"
ping -c 1 192.168.100.1 >/dev/null 2>&1 && echo "✅ Ethernet to Windows PC: Working" || echo "❌ Ethernet to Windows PC: Failed"

echo ""
if command -v nmcli >/dev/null 2>&1; then
    echo "NetworkManager Status:"
    nmcli device status
    
    echo ""
    echo "Available WiFi Networks (first 5):"
    nmcli device wifi list 2>/dev/null | head -6 || echo "WiFi scanning failed"
    
    echo ""
    echo "Active Connections:"
    nmcli connection show --active
else
    echo "NetworkManager CLI not available"
fi

echo ""
echo "Hardware Radio Status:"
sudo rfkill list
EOF

chmod +x /home/$(logname)/ubuntu_network_status.sh

echo "Ubuntu MATE network status script created: ~/ubuntu_network_status.sh"
echo "Run it with: ./ubuntu_network_status.sh"
echo ""
echo "Restore complete! You may want to reboot for all changes to take effect."
echo "sudo reboot"