# 🚀 Deployment Guide - Robot G1 Handover

Przewodnik wdrażania systemu Robot G1 Handover w różnych środowiskach.

---

## 📋 Spis Treści

1. [Typy Deployment](#typy-deployment)
2. [Development Environment](#development-environment)
3. [Testing Environment](#testing-environment)
4. [Production Environment](#production-environment)
5. [Docker Deployment](#docker-deployment)
6. [Cloud Deployment](#cloud-deployment)
7. [Best Practices](#best-practices)

---

## Typy Deployment

### 1. 💻 Development (Deweloperski)
- **Cel**: Rozwój i debugowanie
- **Setup**: Lokalny workspace ROS 2
- **Czas**: 5-30 minut
- **Użycie**: `colcon build` + `source install/setup.bash`

### 2. 🧪 Testing (Testowy)
- **Cel**: Walidacja i testy
- **Setup**: Izolowane środowisko + CI/CD
- **Czas**: 10-60 minut
- **Użycie**: Automated testing pipeline

### 3. 🏭 Production (Produkcyjny)
- **Cel**: Wdrożenie na prawdziwym robocie
- **Setup**: Stabilny system + monitoring
- **Czas**: 1-4 godziny
- **Użycie**: Systemd services + safety features

### 4. 🐳 Docker
- **Cel**: Portable deployment
- **Setup**: Docker container
- **Czas**: 15-45 minut
- **Użycie**: `docker-compose up`

---

## Development Environment

### Wymagania

```bash
# System
Ubuntu 22.04 LTS
Python 3.10+
ROS 2 Humble
Git

# Hardware (minimum)
4GB RAM
10GB disk space
```

### Instalacja Krok po Kroku

#### 1. Przygotowanie Workspace

```bash
# Utwórz workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Sklonuj repozytorium
git clone https://github.com/MatPomGit/robot-g1-Handover.git
```

#### 2. Instalacja Zależności

```bash
cd ~/ros2_ws

# Zależności ROS 2
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Zależności Python
pip3 install -r src/robot-g1-Handover/requirements.txt
```

#### 3. Build

```bash
# Build pakietu
colcon build --packages-select g1_pick_and_handover --symlink-install

# Source workspace
source install/setup.bash

# Dodaj do ~/.bashrc dla automatycznego source'owania
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

#### 4. Weryfikacja

```bash
# Sprawdź czy pakiet jest widoczny
ros2 pkg list | grep g1_pick_and_handover

# Sprawdź executables
ros2 pkg executables g1_pick_and_handover

# Test uruchomienia
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py
```

### Troubleshooting

**Problem**: `colcon build` fails  
**Rozwiązanie**: Sprawdź zależności ROS 2 i Python

**Problem**: `Package not found`  
**Rozwiązanie**: Source workspace: `source install/setup.bash`

---

## Testing Environment

### CI/CD Pipeline (GitHub Actions)

#### Konfiguracja `.github/workflows/ros2_ci.yml`

```yaml
name: ROS 2 CI

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  test:
    runs-on: ubuntu-22.04
    container:
      image: osrf/ros:humble-desktop
    
    steps:
      - uses: actions/checkout@v3
      
      - name: Install dependencies
        run: |
          apt-get update
          rosdep update
          rosdep install --from-paths . --ignore-src -r -y
          pip3 install -r requirements.txt
      
      - name: Build
        run: |
          . /opt/ros/humble/setup.bash
          colcon build --packages-select g1_pick_and_handover
      
      - name: Test
        run: |
          . /opt/ros/humble/setup.bash
          . install/setup.bash
          colcon test --packages-select g1_pick_and_handover
          colcon test-result --verbose
```

### Automated Testing

```bash
# Unit tests
cd ~/ros2_ws
colcon test --packages-select g1_pick_and_handover

# Test results
colcon test-result --verbose

# Coverage
colcon test --packages-select g1_pick_and_handover \
  --event-handlers console_cohesion+ \
  --pytest-args --cov=perception --cov=manipulation --cov=decision
```

---

## Production Environment

### Pre-deployment Checklist

- [ ] System przetestowany w symulacji
- [ ] Safety features zaimplementowane
- [ ] Emergency stop button podłączony
- [ ] Monitoring skonfigurowany
- [ ] Backup system gotowy
- [ ] Dokumentacja wdrożeniowa gotowa
- [ ] Team przeszkolony

### Instalacja Produkcyjna

#### 1. System Setup

```bash
# Na komputerze robota (production machine)

# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble
sudo apt install ros-humble-desktop-full

# Install dependencies
sudo apt install ros-humble-moveit \
                 ros-humble-cv-bridge \
                 ros-humble-vision-msgs
```

#### 2. Production Workspace

```bash
# Utwórz dedykowany workspace
sudo mkdir -p /opt/robot_g1_handover
sudo chown $USER:$USER /opt/robot_g1_handover

cd /opt/robot_g1_handover
mkdir -p src
cd src

# Clone production branch
git clone -b release/v1.0.0 https://github.com/MatPomGit/robot-g1-Handover.git

# Install dependencies
cd /opt/robot_g1_handover
rosdep install --from-paths src --ignore-src -r -y
pip3 install -r src/robot-g1-Handover/requirements.txt

# Build with release flags
colcon build --packages-select g1_pick_and_handover \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### 3. Systemd Service

Utwórz `/etc/systemd/system/robot-g1-handover.service`:

```ini
[Unit]
Description=Robot G1 Handover Service
After=network.target

[Service]
Type=simple
User=robot
WorkingDirectory=/opt/robot_g1_handover
Environment="ROS_DOMAIN_ID=42"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py"
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

Włącz service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable robot-g1-handover.service
sudo systemctl start robot-g1-handover.service

# Sprawdź status
sudo systemctl status robot-g1-handover.service

# Logi
sudo journalctl -u robot-g1-handover.service -f
```

#### 4. Monitoring

##### System Health Check Script

Utwórz `/opt/robot_g1_handover/scripts/health_check.sh`:

```bash
#!/bin/bash

# Health check script for Robot G1 Handover

echo "=== Robot G1 Handover Health Check ==="
echo "Timestamp: $(date)"
echo ""

# Check if service is running
echo "1. Service Status:"
systemctl is-active robot-g1-handover.service && echo "✅ Service is running" || echo "❌ Service is down"
echo ""

# Check ROS 2 nodes
echo "2. ROS 2 Nodes:"
source /opt/ros/humble/setup.bash
source /opt/robot_g1_handover/install/setup.bash

NODES=$(ros2 node list 2>/dev/null | wc -l)
echo "Active nodes: $NODES"
if [ $NODES -gt 5 ]; then
    echo "✅ All nodes running"
else
    echo "⚠️ Some nodes may be down"
fi
echo ""

# Check topics
echo "3. Critical Topics:"
TOPICS=(
    "/camera/color/image_raw"
    "/object_detections"
    "/human_hand_pose"
)

for topic in "${TOPICS[@]}"; do
    timeout 2 ros2 topic hz $topic >/dev/null 2>&1 && \
        echo "✅ $topic" || \
        echo "❌ $topic"
done
echo ""

# Check CPU/Memory
echo "4. Resource Usage:"
echo "CPU: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}')%"
echo "Memory: $(free -m | awk 'NR==2{printf "%.1f%%", $3*100/$2}')"
echo ""

echo "=== Health Check Complete ==="
```

Dodaj do crontab (check co 5 minut):

```bash
chmod +x /opt/robot_g1_handover/scripts/health_check.sh

# Crontab
crontab -e
# Dodaj linię:
# */5 * * * * /opt/robot_g1_handover/scripts/health_check.sh >> /var/log/robot_health.log 2>&1
```

#### 5. Safety Features

##### Emergency Stop Integration

Dodaj do launch file:

```python
# full_handover_pipeline.launch.py

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    # Emergency stop monitor
    emergency_stop_node = Node(
        package='g1_pick_and_handover',
        executable='emergency_stop_monitor',
        name='emergency_stop',
        parameters=[{
            'gpio_pin': 17,  # GPIO pin dla przycisku E-stop
            'check_rate': 50.0  # 50 Hz
        }],
        output='screen'
    )
    
    # Shutdown on emergency stop
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=emergency_stop_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )
    
    return LaunchDescription([
        emergency_stop_node,
        shutdown_handler,
        # ... inne node'y
    ])
```

---

## Docker Deployment

### Dockerfile

Utwórz `Dockerfile`:

```dockerfile
FROM osrf/ros:humble-desktop

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-moveit \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /opt/ros2_ws
COPY . /opt/ros2_ws/src/robot-g1-Handover

# Install Python dependencies
RUN pip3 install -r src/robot-g1-Handover/requirements.txt

# Build workspace
RUN . /opt/ros/humble/setup.bash && \
    colcon build --packages-select g1_pick_and_handover

# Source workspace in entrypoint
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros2_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c"]
CMD ["source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py"]
```

### Docker Compose

Utwórz `docker-compose.yml`:

```yaml
version: '3.8'

services:
  robot-g1-handover:
    build: .
    image: robot-g1-handover:1.0.0
    container_name: robot-g1-handover
    network_mode: host
    privileged: true
    volumes:
      - /dev:/dev  # Dostęp do devices (kamera, etc.)
      - ./logs:/opt/ros2_ws/logs
    environment:
      - ROS_DOMAIN_ID=42
      - DISPLAY=${DISPLAY}
    restart: unless-stopped
```

### Użycie Docker

```bash
# Build image
docker-compose build

# Start services
docker-compose up -d

# View logs
docker-compose logs -f

# Stop services
docker-compose down

# Shell access
docker exec -it robot-g1-handover bash
```

---

## Cloud Deployment

### AWS EC2 (Przykład)

#### 1. Launch EC2 Instance

```bash
# Instance type: g4dn.xlarge (z GPU dla YOLO)
# AMI: Ubuntu 22.04 LTS
# Storage: 50GB EBS
# Security group: SSH (22), ROS 2 DDS ports
```

#### 2. Setup na EC2

```bash
# SSH do instancji
ssh -i key.pem ubuntu@<EC2_IP>

# Install ROS 2
# ... (jak w Production Environment)

# Install NVIDIA drivers (dla GPU)
sudo apt install nvidia-driver-525

# Deploy aplikacji
# ... (jak w Production Environment)
```

#### 3. Load Balancing (opcjonalnie)

Dla wielu instancji, użyj AWS Load Balancer.

---

## Best Practices

### 1. Version Control

```bash
# Use semantic versioning
git tag -a v1.0.0 -m "Release v1.0.0"
git push origin v1.0.0

# Deploy z konkretnej wersji
git clone -b v1.0.0 https://github.com/MatPomGit/robot-g1-Handover.git
```

### 2. Configuration Management

```bash
# Użyj environment-specific configs
config/
  ├── development.yaml
  ├── testing.yaml
  └── production.yaml

# Load config w launch file
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py \
  config_file:=/path/to/production.yaml
```

### 3. Logging

```python
# Structured logging
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/var/log/robot_g1_handover.log'),
        logging.StreamHandler()
    ]
)
```

### 4. Monitoring & Alerting

```bash
# Use Prometheus + Grafana
# Export ROS 2 metrics
ros2 run prometheus_ros2 metrics_exporter
```

### 5. Backup Strategy

```bash
# Daily backup script
#!/bin/bash
BACKUP_DIR=/backup/robot_g1_handover/$(date +%Y%m%d)
mkdir -p $BACKUP_DIR

# Backup workspace
tar -czf $BACKUP_DIR/workspace.tar.gz /opt/robot_g1_handover

# Backup configs
cp -r /opt/robot_g1_handover/src/robot-g1-Handover/config $BACKUP_DIR/

# Backup logs
cp -r /var/log/robot_g1_handover $BACKUP_DIR/

# Keep last 7 days
find /backup/robot_g1_handover -type d -mtime +7 -exec rm -rf {} \;
```

### 6. Security

```bash
# Firewall
sudo ufw enable
sudo ufw allow 22/tcp  # SSH
sudo ufw allow from 192.168.1.0/24  # Local network dla ROS 2

# User permissions
sudo useradd -m -s /bin/bash robot
sudo usermod -aG dialout robot  # Serial ports
sudo usermod -aG video robot    # Cameras
```

---

## 🆘 Troubleshooting

### Problem: Service nie startuje

```bash
# Sprawdź logi
sudo journalctl -u robot-g1-handover.service -n 50

# Sprawdź permissions
ls -la /opt/robot_g1_handover

# Test manualny
cd /opt/robot_g1_handover
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch g1_pick_and_handover full_handover_pipeline.launch.py
```

### Problem: High CPU usage

```bash
# Profiling
ros2 run ros2_profiler cpu_profiler

# Obniż FPS kamery
ros2 param set /camera/realsense2_camera fps 15

# Użyj lżejszego modelu YOLO
ros2 param set /object_detector model_name yolov5n  # nano model
```

---

## 📞 Support

Problemy z deploymentem?

- **GitHub Issues**: https://github.com/MatPomGit/robot-g1-Handover/issues
- **Email**: contact@robotg1handover.org
- **Dokumentacja**: [README.md](README.md), [TROUBLESHOOTING.md](TROUBLESHOOTING.md)

---

<div align="center">

**Pomyślnego wdrożenia!** 🚀🤖

**[⬆ Powrót do góry](#-deployment-guide---robot-g1-handover)**

</div>
