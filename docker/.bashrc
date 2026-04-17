export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

GLOBAL_ROS_SETUP="/opt/ros/humble/setup.bash"
echo "Init ros install from $GLOBAL_ROS_SETUP"
source "$GLOBAL_ROS_SETUP"

LOCAL_ROS_SETUP="$HOME/local/workspace/ws/install/setup.bash"
if test -f "$LOCAL_ROS_SETUP"; then 
    echo "Init local ros workspace from $LOCAL_ROS_SETUP"
    # Uncomment this line after workspace is built
    # source "$LOCAL_ROS_SETUP"
else
    echo "Local ros workspace has not been built."
    echo "Build workspace and source it." 
fi
export ROS_DOMAIN_ID=50
cd $HOME/local/workspace/ws

