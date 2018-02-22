#!/bin/bash

echo "Download, Build  and run Main Control"
echo "====================================="
echo "Updating download & build script..."
cp /mnt/dev/OPi_MainConrol/build.sh . &&
echo "Launching script..." &&
./build.sh
# echo "Launching program..." &&
# ./sunxi_gpio

