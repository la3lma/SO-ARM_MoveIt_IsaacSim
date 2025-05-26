#!/bin/bash

echo "Building Docker image to find minimal dependencies..."
docker build -f Dockerfile.minimal -t so-arm-deps-test . 2>&1 | tee build.log

echo ""
echo "=== EXTRACTING DEPENDENCIES ==="
echo "Check build.log for the 'ROSDEP SIMULATION' section"
echo "Those are your true minimal dependencies!"

# Extract just the apt install commands
echo ""
echo "=== APT INSTALL COMMAND ==="
grep "apt-get install" build.log | head -1 