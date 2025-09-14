# Smart IoT prepaid charger for light EV in low and middle income countries (prepaid prototype)

**What it does:** Phone UI starts a charging session with a Wh budget. The app reads a PZEM-017 over RS-485, integrates power (W×Δt), and cuts off at the target. Works on Raspberry Pi 5 (no Flask).

## Hardware
- Raspberry Pi 5
- Waveshare SIM7670G
- PZEM-017 (Modbus RTU), powered and **common GND** with the Pi
- BD139 BJT NPN transistor with base on **GPIO18 (BCM)** to control the charger path

## Install on a fresh Pi
bash
git clone https://github.com/Global-Health-Engineering/rfid-controlled-charging.git ~/rfid-controlled-charging
cd ~/rfid-controlled-charging
bash src/scripts/setup.sh

# Optional: remote access with Tailscale (recommended)
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up --ssh
tailscale ip -4

