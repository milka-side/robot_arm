# Docker

## Dependencies
```bash
curl -fsSL https://get.docker.com | sh  # installs docker + compose plugin
sudo usermod -aG docker $USER           # run docker without sudo (re-login required)
```

## Quick Reference

| Action | Command |
|--------|---------|
| Build | `docker compose build` |
| Create + start | `docker compose up -d` |
| Enter container | `docker compose exec robot_arm_dev bash` |
| Stop (keep container) | `docker compose stop` |
| Start stopped container | `docker compose start` |
| Stop + delete | `docker compose down` |

> You can enter the same container from multiple terminals simultaneously —
> that's how you run Gazebo/RViz/gamepad teleop each in their own terminal.

## Display Access (for GUI / Gazebo / RViz)

Allow Docker to use your screen before entering the container:

```bash
xhost +local:docker
```

## Troubleshooting

* Commands not found? Try `docker-compose` (with -) or prepend sudo.
* Container already exists? `docker compose up -d` will just start it, not recreate.
* `Error: could not select device driver "nvidia" with capabilities: [[gpu]]` —
  install the NVIDIA Container Toolkit so Docker can bridge to your host's
  NVIDIA drivers (host drivers must already be installed):
  ```bash
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
  sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
  sudo nvidia-ctk runtime configure --runtime=docker
  sudo systemctl restart docker
  ```
