#!/bin/bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <compressed|ffmpeg> <topic> [topic ...]" >&2
  exit 2
fi

transport="$1"
shift

case "$transport" in
  compressed|ffmpeg)
    ;;
  *)
    echo "Unsupported transport '${transport}'. Expected 'compressed' or 'ffmpeg'." >&2
    exit 2
    ;;
esac

png_level="${COMPRESS_PNG_LEVEL:-1}"
ffmpeg_encoder="${COMPRESS_FFMPEG_ENCODER:-libx264}"
ffmpeg_encoder_av_options="${COMPRESS_FFMPEG_ENCODER_AV_OPTIONS:-preset:ultrafast,tune:zerolatency,crf:0}"
ffmpeg_bit_rate="${COMPRESS_FFMPEG_BIT_RATE:-20000000}"
ffmpeg_gop_size="${COMPRESS_FFMPEG_GOP_SIZE:-10}"
ffmpeg_qmax="${COMPRESS_FFMPEG_QMAX:-1}"

pids=()

cleanup() {
  local pid=""
  for pid in "${pids[@]:-}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done
  for pid in "${pids[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done
}

trap cleanup EXIT INT TERM

for topic in "$@"; do
  [[ -n "${topic}" ]] || continue
  topic_suffix="${topic#/}"
  topic_suffix="${topic_suffix//\//_}"

  republish_cmd=(
    ros2 run image_transport republish --ros-args
    -r __node:="image_republisher_${transport}_${topic_suffix}"
    -p in_transport:=raw
    -p out_transport:="${transport}"
    -r in:="${topic}"
    -p "qos_overrides.${topic}.subscription.reliability:=best_effort"
    -p "qos_overrides.${topic}.subscription.durability:=volatile"
  )

  if [[ "${transport}" == "compressed" ]]; then
    republish_cmd+=(
      -r /out/compressed:="${topic}/compressed"
      -p out.compressed.format:=png
      -p out.compressed.png_level:="${png_level}"
    )
  else
    republish_cmd+=(
      -r /out/ffmpeg:="${topic}/ffmpeg"
      -p out.ffmpeg.encoder:="${ffmpeg_encoder}"
      -p out.ffmpeg.encoder_av_options:="${ffmpeg_encoder_av_options}"
      -p out.ffmpeg.bit_rate:="${ffmpeg_bit_rate}"
      -p out.ffmpeg.gop_size:="${ffmpeg_gop_size}"
      -p out.ffmpeg.qmax:="${ffmpeg_qmax}"
    )
  fi

  echo "Starting ${transport} republisher for ${topic}"
  "${republish_cmd[@]}" &
  pids+=("$!")
done

wait -n "${pids[@]}"
