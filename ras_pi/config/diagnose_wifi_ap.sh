#!/bin/bash
# WiFi Access Point Diagnostic Script for Raspberry Pi
# This script diagnoses why devices cannot connect to the AP

set -e

echo "========================================="
echo "WiFi Access Point Diagnostic Tool"
echo "========================================="
echo ""

# Function to print section headers
print_section() {
    echo ""
    echo "--- $1 ---"
}

# Function to check command success
check_result() {
    if [ $? -eq 0 ]; then
        echo "✅ $1: OK"
    else
        echo "❌ $1: FAILED"
    fi
}

print_section "1. Service Status Check"

# Check hostapd service
echo "Checking hostapd service..."
sudo systemctl is-active hostapd >/dev/null 2>&1
check_result "hostapd service active"

sudo systemctl is-enabled hostapd >/dev/null 2>&1
check_result "hostapd service enabled"

echo "hostapd detailed status:"
sudo systemctl status hostapd --no-pager -l

print_section "2. dnsmasq Service Check"

# Check dnsmasq service
echo "Checking dnsmasq service..."
sudo systemctl is-active dnsmasq >/dev/null 2>&1
check_result "dnsmasq service active"

sudo systemctl is-enabled dnsmasq >/dev/null 2>&1
check_result "dnsmasq service enabled"

echo "dnsmasq detailed status:"
sudo systemctl status dnsmasq --no-pager -l

print_section "3. Network Interface Check"

# Check wlan0 interface
echo "Checking wlan0 interface..."
ip link show wlan0 >/dev/null 2>&1
check_result "wlan0 interface exists"

echo "wlan0 interface details:"
ip addr show wlan0

echo "wlan0 wireless info:"
iw dev wlan0 info

print_section "4. Configuration Files Check"

# Check hostapd config
echo "Checking hostapd configuration..."
if [ -f /etc/hostapd/hostapd.conf ]; then
    echo "✅ hostapd.conf exists"
    echo "hostapd.conf contents:"
    cat /etc/hostapd/hostapd.conf
else
    echo "❌ hostapd.conf missing"
fi

# Check dnsmasq config
echo ""
echo "Checking dnsmasq configuration..."
if [ -f /etc/dnsmasq.conf ]; then
    echo "✅ dnsmasq.conf exists"
    echo "dnsmasq.conf contents:"
    cat /etc/dnsmasq.conf
else
    echo "❌ dnsmasq.conf missing"
fi

print_section "5. Process Check"

# Check if processes are running
echo "Checking running processes..."
pgrep hostapd >/dev/null 2>&1
check_result "hostapd process running"

pgrep dnsmasq >/dev/null 2>&1
check_result "dnsmasq process running"

echo "Process details:"
ps aux | grep -E "(hostapd|dnsmasq)" | grep -v grep

print_section "6. Network Conflicts Check"

# Check for conflicting services
echo "Checking for NetworkManager conflicts..."
systemctl is-active NetworkManager >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "⚠️  NetworkManager is active - may conflict with hostapd"
    echo "NetworkManager managed devices:"
    nmcli device status 2>/dev/null || echo "nmcli not available"
else
    echo "✅ NetworkManager not active"
fi

echo ""
echo "Checking for wpa_supplicant conflicts..."
pgrep wpa_supplicant >/dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "⚠️  wpa_supplicant is running - may conflict with hostapd"
    ps aux | grep wpa_supplicant | grep -v grep
else
    echo "✅ wpa_supplicant not running"
fi

print_section "7. Port and Socket Check"

# Check if required ports are available
echo "Checking port usage..."
echo "Port 53 (DNS):"
sudo lsof -i :53 2>/dev/null || echo "Port 53 not in use"

echo "Port 67 (DHCP):"
sudo lsof -i :67 2>/dev/null || echo "Port 67 not in use"

print_section "8. Firewall Check"

# Check iptables rules
echo "Checking iptables rules..."
echo "NAT table POSTROUTING rules:"
sudo iptables -t nat -L POSTROUTING

echo "Filter table FORWARD rules:"
sudo iptables -L FORWARD

print_section "9. Log Analysis"

# Check recent logs for errors
echo "Recent hostapd logs (last 20 lines):"
sudo journalctl -u hostapd -n 20 --no-pager

echo ""
echo "Recent dnsmasq logs (last 20 lines):"
sudo journalctl -u dnsmasq -n 20 --no-pager

print_section "10. WiFi Scan Test"

# Test if AP is visible
echo "Scanning for your AP..."
sudo iw dev wlan0 scan 2>/dev/null | grep -A 10 -B 2 "MultiRobot_Network" || echo "AP not visible in scan"

print_section "11. Authentication Test"

# Check for authentication issues
echo "Testing hostapd configuration syntax..."
sudo hostapd -t /etc/hostapd/hostapd.conf
check_result "hostapd config syntax"

print_section "12. Recommendations"

echo ""
echo "Common issues and solutions:"
echo ""
echo "1. If hostapd shows 'Could not configure driver mode':"
echo "   - Run: sudo rfkill unblock wifi"
echo "   - Run: sudo nmcli radio wifi on"
echo ""
echo "2. If devices can see but can't connect:"
echo "   - Check WPA passphrase in /etc/hostapd/hostapd.conf"
echo "   - Verify channel compatibility (try channel 1, 6, or 11)"
echo "   - Check if country code is set: sudo raspi-config > Localisation"
echo ""
echo "3. If DHCP not working:"
echo "   - Check dnsmasq is running: sudo systemctl restart dnsmasq"
echo "   - Verify IP range doesn't conflict with existing networks"
echo ""
echo "4. If authentication fails:"
echo "   - Try changing WPA2 to WPA/WPA2 mixed mode"
echo "   - Check password length (8-63 characters)"
echo ""
echo "5. Manual restart sequence:"
echo "   sudo systemctl stop hostapd dnsmasq"
echo "   sudo ip link set wlan0 down"
echo "   sudo ip link set wlan0 up"
echo "   sudo systemctl start hostapd dnsmasq"

echo ""
echo "========================================="
echo "Diagnostic complete!"
echo "========================================="
