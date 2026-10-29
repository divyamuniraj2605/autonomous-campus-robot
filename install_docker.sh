#!/usr/bin/env bash
# Run once on the host. Afterwards LOG OUT of Ubuntu and log back in.
sudo apt update && sudo apt install -y docker.io
sudo usermod -aG docker "$USER"
echo "Done. Log out and back in, then run ./run_demo.sh"
