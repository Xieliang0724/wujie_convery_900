#!/usr/bin/env bash
# Flash the built HEX to the GD32F407VG board via SEGGER J-Link.
set -euo pipefail
HEX="${1:-out/wujie_convery/Debug/Debug/wujie_convery.hex}"
if [ ! -f "$HEX" ]; then
  echo "找不到固件 $HEX，请先运行 ./build.sh 编译" >&2
  exit 1
fi
printf 'device STM32F407VG\nsi SWD\nspeed 4000\nconnect\nloadfile "%s"\nr\ng\nexit\n' "$HEX" | /usr/local/bin/JLinkExe
