# Remember to exi out of all python/conda env and remove every existing interbotix_ws folders and april_tag folder
# don't need to install ros humble, the script will automatically install it
# source "$(dirname "$0")/disable-conda.bashrc"
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
bash ./xsarm_amd64_install.sh -d humble
rm -rf ./xsarm_amd64_install.sh