#!/usr/bin/env bash
# Parent Python supervisor retains the port lock for the complete owned round.
set -euo pipefail
# Setup scripts must not inherit launcher flags such as --help.
cane_round_args=("$@")
set --
source /home/xcg/ws/devel/setup.bash
source "$HOME/cane-pedsim-ws/devel/setup.bash" --extend
pkg="$(rospack find plan_manage)"
exec /usr/bin/python3 "$pkg/scripts/pedsim_round.py" "${cane_round_args[@]}"
