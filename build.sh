#!/usr/bin/env bash
# Build wujie_convery with CMSIS-Toolbox + Arm GNU Toolchain on macOS.
set -euo pipefail

export CMSIS_PACK_ROOT="$HOME/.cache/arm/packs"
export ARM_GNU="$HOME/arm-gnu-toolchain/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi/bin"
export GCC_TOOLCHAIN_15_2_1="$ARM_GNU"
export PATH="$HOME/cmsis-toolbox/cmsis-toolbox-darwin-arm64/bin:$ARM_GNU:/opt/homebrew/bin:$PATH"

cd "$(dirname "$0")"
cbuild wujie_convery.csolution.yml "$@"
