#!/bin/zsh
cd "${0:A:h}"
python3 flash_usb.py --flash
result=$?
echo "Finished with status $result. Press Return to close."
read
