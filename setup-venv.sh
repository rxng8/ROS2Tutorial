# Disable conda if needed
# source "$(dirname "$0")/misc/disable-conda.bashrc"

# Create virtual environment
sudo apt install -y python3-virtualenv
# virtualenv -p python3 ./.venv
# virtualenv --python python3.11 ./.venv # Ros2 humble does not support python 3.11 (for now), the official support is python 3.10
virtualenv --python python3.10 ./.venv
# Let colcon knw that we are not building this folder
touch ./.venv/COLCON_IGNORE