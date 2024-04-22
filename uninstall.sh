#!/usr/bin/env bash

# Colors
NC='\033[0m'          # Text Reset
On_Black='\033[40m'   # Black background
On_Purple='\033[45m'  # Purple background
On_Cyan='\033[46m'    # Cyan background
On_Green='\033[42m'   # Green background
BIRed='\033[1;91m'    # Bold Intense Red
BWhite='\033[1;37m'   # Bold White
BIYellow='\033[1;93m' # Bold Intense Yellow
White='\033[0;37m'    # White

# Printing utilities
ColErr=${BIRed}${On_Black}
ColPrompt=${BWhite}${On_Cyan}
ColInfo=${White}${On_Purple}
ColWarn=${BIYellow}${On_Black}

cout() {
  echo -e "${ColInfo}${1}${NC}"
}
cin() {
  echo -e "${ColPrompt}${1}${NC}"
}
cerr() {
  echo -e "${ColErr}${1}${NC}"
}
cwarn() {
  echo -e "${ColWarn}${1}${NC}"
}
ctitle() {
  echo -e "${BWhite}${On_Green}${1}${NC}"
}

# Local functions
promptYesNo() {
  local REPLY=${2}
  local TXT="[y/n]"

  if [[ "$REPLY" == 1 ]]; then
    TXT="[Y/n]"
  elif [[ "$REPLY" == 0 ]]; then
    TXT="[y/N]"
  else
    REPLY=""
  fi

  while true; do
    local color=$'\033[1;37m\033[46m'
    local noColor=$'\033[0m'
    read -r -e -p "$color${1} $TXT?$noColor " yn

    case $yn in
    y | Y | Yes | yes | YES)
      REPLY=1
      break
      ;;
    n | N | No | no | NO)
      REPLY=0
      break
      ;;
    "")
      if [ -n "$REPLY" ]; then
        break
      fi
      ;;
    *) ;;
    esac
  done

  echo "$REPLY"
}

cout "Removing cleanly lll-supervisor"

# Check if the service exists and remove it
if [ -f "$HOME/.config/systemd/user/lll_control_panel.service" ]; then
  systemctl --user stop lll_control_panel
  systemctl --user disable lll_control_panel
  rm -f "$HOME/.config/systemd/user/lll_control_panel.service"
  systemctl --user daemon-reload
fi || true

SUDO=""
if [[ "$EUID" -ne 0 ]]; then
  cwarn "To install the supervisor automatically some commands must be run as sudo"
  SUDO="sudo "
fi || true

if ! ($SUDO apt remove lll-supervisor* -y); then
  cerr "Error: Failed to fully remove lll-supervisor"
fi

launch_cache_folder=/opt/ros/humble/share/lll_supervisor/launch
if [ -d "$launch_cache_folder" ]; then
  echo "Deleting folder cache $launch_cache_folder"
  rm -rf "$launch_cache_folder"
fi || true

config_folder=$HOME/.3laws
DELETE_FOLDER=$(promptYesNo "Do you want to remove 3laws config directory \$HOME/.3laws ?" 1)
if [ -d "$config_folder" ] && [ "$DELETE_FOLDER" ]; then
  echo "Deleting config folder $config_folder"
  rm -rf "$config_folder"
fi || true

cout "lll-supervisor has been removed"
