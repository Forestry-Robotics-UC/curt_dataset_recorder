#!/bin/bash

set -u

MAX_RETRIES=1
TIMEOUT=40

send_goal() {
    local x=$1
    local y=$2
    local qz=$3
    local qw=$4

    local attempt=1

    while [ $attempt -le $MAX_RETRIES ]; do

        echo "=================================================="
        echo "Goal: ($x, $y)  Attempt $attempt/$MAX_RETRIES"
        echo "=================================================="

        output=$(timeout ${TIMEOUT}s ros2 action send_goal \
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
                    z: $qz,
                    w: $qw
                  }
                }
              }
            }" --feedback 2>&1)

        #echo "$output"

        if echo "$output" | grep -q "Goal finished with status: SUCCEEDED"; then
            echo "Goal reached successfully"
            return 0
        fi

        echo "Goal failed or timed out"

        if [ $attempt -lt $MAX_RETRIES ]; then
            echo "Retrying goal..."
            sleep 3
        fi

        ((attempt++))

    done

    echo "Skipping goal after $MAX_RETRIES failed attempts"
    return 1
}


#########################################################
# Navigation mission
#########################################################

echo "Starting navigation mission..."

# Goal 1
send_goal -10.2 8.2 0.0 1.0

# Goal 2
send_goal -9.7 23.4 0.0 1.0

# Goal 3
send_goal -10.2 8.2 0.0 1.0

# Goal 4
send_goal 0.0 0.0 0.0 1.0


echo "=================================================="
echo "Mission finished"
echo "=================================================="