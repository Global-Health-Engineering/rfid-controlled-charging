#!/usr/bin/env bash
set -euo pipefail

# System packages
sudo apt-get update
sudo apt-get install -y python3 python3-pip python3-rpi.gpio pinctrl git

# Optional: create a venv and install Python deps
# (uncomment this block if you want to run via venv)
# python3 -m venv /home/acs/charger-venv
# source /home/acs/charger-venv/bin/activate
# pip install --upgrade pip
# pip install -r /home/acs/rfid-controlled-charging/src/requirements.txt
# deactivate

# Serial access for your user (log out/in once after running)
sudo usermod -a -G dialout "$USER" || true

# Install and enable the systemd service
sudo install -m0644 /home/acs/rfid-controlled-charging/src/systemd/charger-app.service /etc/systemd/system/charger-app.service
sudo systemctl daemon-reload
sudo systemctl enable --now charger-app

echo "Done. If SERIAL_PORT differs on this Pi, edit /home/acs/rfid-controlled-charging/src/charger_app.py and restart: sudo systemctl restart charger-app"
