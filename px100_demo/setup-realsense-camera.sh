# # Installation guide: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
# sudo apt install curl -y
# sudo mkdir -p /etc/apt/keyrings
# curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
# sudo apt-get install -y apt-transport-https
# echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
# sudo tee /etc/apt/sources.list.d/librealsense.list
# sudo apt-get update -y
# sudo apt-get install -y librealsense2-dkms
# sudo apt-get install -y librealsense2-utils
# modinfo uvcvideo | grep "version:"
# # More information about uninstalling, updating, etc., refer to the link on top.


# Installation guide from here, install it directly: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md