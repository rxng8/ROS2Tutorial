source ~/.bashrc

# Deactivate cond first
# conda activate base
# conda deactivate

# Basically source everything
source ./.venv/bin/activate # virtual env python

source /opt/ros/humble/setup.bash # Original ros env
source ~/interbotix_ws/install/setup.bash # the interbotix robot ws
# source "$(dirname "$(dirname "$(dirname "$0" )")")/px100/install/setup.bash" # Remember to build your px100 package first using `cs px100 && colcon build`. NOTE: Have weird error
# source "./px100/controller/install/setup.bash" # Remember to build your px100 package first using `cs px100 && colcon build`. Have to do it from the root directory of this project

