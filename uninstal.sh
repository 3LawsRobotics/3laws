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

cout "Removing cleanly lll-supervisor"

# Check if the service exists and remove it
if [ -f "$HOME/.config/systemd/user/lll_control_panel.service" ]; then
  systemctl --user stop backup
  systemctl --user disable backup
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
