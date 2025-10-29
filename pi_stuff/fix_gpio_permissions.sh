#!/bin/bash
# Fix GPIO permissions for Raspberry Pi

echo "=========================================="
echo "Fixing GPIO Permissions"
echo "=========================================="

# Add user to gpio group
echo "Adding user to gpio group..."
sudo usermod -a -G gpio $USER

# Check if user is in gpio group
if groups $USER | grep -q "\bgpio\b"; then
    echo "✅ User is now in gpio group"
else
    echo "❌ Failed to add user to gpio group"
fi

# Set proper permissions for GPIO devices
echo "Setting GPIO device permissions..."
sudo chmod 666 /dev/gpiomem 2>/dev/null || echo "⚠️  /dev/gpiomem not found (normal on some Pi models)"

# Alternative: run with sudo
echo ""
echo "Alternative solutions:"
echo "1. Log out and log back in, then try: python3 tim240_run.py"
echo "2. Or run with sudo: sudo python3 tim240_run.py"
echo "3. Or use: sudo -E python3 tim240_run.py (preserves environment)"

echo ""
echo "=========================================="
echo "GPIO Permission Fix Complete"
echo "=========================================="
