#!/bin/bash

set -e

send_goal() {
  local x=$1
  local y=$2
  local z=$3
  local w=$4

  echo "Navigating to: ($x, $y)"

  ros2 action send_goal \
    /navigate_to_pose \
    nav2_msgs/action/NavigateToPose \
    "{
      pose: {
        header: {
          frame_id: \"map_curt\"
        },
        pose: {
          position: {
            x: $x,
            y: $y,
            z: 0.0
          },
          orientation: {
            x: 0.0,
            y: 0.0,
            z: $z,
            w: $w
          }
        }
      }
    }"

  echo "Goal complete."
  echo
}

# Goal 1
send_goal -10.2 8.2 0.0 1.0

# Goal 2
send_goal -9.7 23.4 0.0 1.0

# Goal 3
send_goal -10.2 8.2 0.0 1.0

# Goal 4
send_goal 0.0 0.0 0.0 1.0

echo "All goals completed."