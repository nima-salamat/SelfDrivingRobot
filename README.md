
---

# **Self Driving Robot**  


## **📦 Dependencies**  


```bash
# Update system
sudo apt update
sudo apt upgrade -y

# Install core packages
sudo apt install -y python3-pip python3-opencv

# Install pyusb and backind library
sudo apt-get install libusb-1.0-0-dev
pip install pyusb
# For Pi Camera support:
sudo apt install -y python3-pycamera2
sudo raspi-config  # Enable Camera: Interface Options > Camera > Enable

# For AprilTags detection:
pip3 install apriltag numpy
```
---

## **🚀 Quick Start**  
1. Clone this repository:  
   ```bash
   git clone https://github.com/nima-salamat/SelfDrivingRobot.git
   cd SelfDrivingRobot
   ```

2. Run the main script:  
   ```bash
   python3 main.py
   ```

---

