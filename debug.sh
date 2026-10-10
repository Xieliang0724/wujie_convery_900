#!/usr/bin/env bash
# Debug wujie_convery via SEGGER J-Link GDB Server + GNU GDB.
set -euo pipefail
ARM_GNU="$HOME/arm-gnu-toolchain/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi/bin"
ELF="out/wujie_convery/Debug/Debug/wujie_convery.elf"
if [ ! -f "$ELF" ]; then
  echo "找不到 $ELF，请先运行 ./build.sh 编译" >&2
  exit 1
fi
/usr/local/bin/JLinkGDBServer -device STM32F407VG -if SWD -speed 4000 -port 3333 >/tmp/jlink_gdb.log 2>&1 &
SERVER_PID=$!
cleanup(){ kill "$SERVER_PID" 2>/dev/null || true; }
trap cleanup EXIT
sleep 1
"$ARM_GNU/arm-none-eabi-gdb" "$ELF" \
  -ex "target extended-remote :3333" \
  -ex "monitor reset" \
  -ex "load" \
  -ex "monitor go"
