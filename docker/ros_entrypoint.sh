#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
# source "/tmp/workspace/devel/setup.bash"
exec "$@"
