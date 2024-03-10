#!/bin/bash

set -e

source /opt/ros/humble/setup.bash

source /diff_drive_bot/install/setup.bash

echo "Provided arguments: $@"

exec $@