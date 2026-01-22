#!/usr/bin/env sh
source install/setup.zsh
echo "Sourcing setup files for the current shell: $SHELL"
case "$SHELL" in
  */bash)
    source install/setup.bash
    ;;
  */zsh)
    source install/setup.zsh
    ;;
  *)
    echo "Unknown shell: $SHELL"
