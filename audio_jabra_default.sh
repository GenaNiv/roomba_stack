#!/usr/bin/env bash
set -euo pipefail

# restart user audio stack (safe)
systemctl --user restart pipewire pipewire-pulse wireplumber >/dev/null 2>&1 || true
sleep 1

SINK=$(pactl list short sinks   | awk '/Jabra|SPEAK/ && /analog-stereo/ {print $2; exit}')
SOURCE=$(pactl list short sources | awk '/Jabra|SPEAK/ && !/\.monitor/ {print $2; exit}')

if [[ -z "${SINK:-}" || -z "${SOURCE:-}" ]]; then
  echo "Jabra sink/source not found. Is it plugged in?" >&2; exit 1
fi

pactl set-default-sink   "$SINK"
pactl set-default-source "$SOURCE"
pactl set-sink-mute      @DEFAULT_SINK@   0
pactl set-sink-volume    @DEFAULT_SINK@   75%
pactl set-source-mute    @DEFAULT_SOURCE@ 0
pactl set-source-volume  @DEFAULT_SOURCE@ 75%

echo "Default sink   = $SINK"
echo "Default source = $SOURCE"
