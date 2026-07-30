#!/bin/bash

# Configuration
FILE_TIMESTAMP=$(date +"%Y-%m-%d_%H:%M:%S")
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
OUTPUT_FILE="streaming_stats_curt_$FILE_TIMESTAMP.csv"
INTERVAL=1

# Add headers if file doesn't exist
if [ ! -f "$OUTPUT_ILE" ]; then
    echo "Timestamp,Name,CPU_Perc,Mem_Usage,Mem_Perc,Net_IO,Block_IO,PIDs" > "$SCRIPT_DIR/monitor_stats/$OUTPUT_FILE"
fi

echo "Streaming docker stats to $OUTPUT_FILE... Press [CTRL+C] to stop."

# Stream loop
while true; do
    TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")

    docker stats --no-stream --format "$TIMESTAMP,{{.Name}},{{.CPUPerc}},{{.MemUsage}},{{.MemPerc}},{{.NetIO}},{{.BlockIO}},{{.PIDs}}" | tee -a "$SCRIPT_DIR/monitor_stats/$OUTPUT_FILE"

    sleep "$INTERVAL"
done