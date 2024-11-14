<a name="readme-top"></a>

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://website.com">
    <img src="https://i.imgur.com/qqK3as2.png" alt="Logo" width="357" height="162">
  </a>

<h3 align="center">A Framework to Enable Human-Drone Interaction Through Natural Hand Gesture Control</h3>

  <p align="center">
    Design Considerations for Stability and Efficiency in Unmanned Aerial Vehicle
    <br />
    <a href="https://github.com/yuriachermann/Hand-Controlled-PX4-Drone"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://website.com">View Demo</a>
    ·
    <a href="https://github.com/yuriachermann/Hand-Controlled-PX4-Drone/issues">Report Bug</a>
    ·
    <a href="https://github.com/yuriachermann/Hand-Controlled-PX4-Drone/issues">Request Feature</a>
  </p>
</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <!-- <li><a href="#roadmap">Roadmap</a></li> -->
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

<!-- ABOUT THE PROJECT -->
## About The Project

This project introduces a novel framework to enhance human-drone interaction through natural hand gesture controls. Using a Leap Motion Controller, this system enables intuitive drone operation by translating hand gestures into flight commands. The integration of ROS2, PX4, and a companion computer allows for a seamless connection between the Ground Control Station (GCS) and the drone. The system is validated in a SITL Gazebo simulation environment to refine and test the approach before real-world deployment.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Built With

* [![Python][Python]][Python-url]
* [![Ubuntu][Ubuntu]][Ubuntu-url]
* [![ROS2][ROS2]][ROS2-url]
* [![PX4][PX4]][PX4-url]
* [![Gazebo][Gazebo]][Gazebo-url]
* [![QGroundControl][QGroundControl]][QGroundControl-url]
* [![Ultraleap][Ultraleap]][Ultraleap-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple example steps.

### Prerequisites

* **Software**:
  * [Ubuntu 22.04](https://ubuntu.com/download/alternative-downloads)
  * [ROS2 Humble](https://control.ros.org/humble/index.html)
  * [Python 3.10](https://www.python.org/downloads/release/python-31011/)
  * [PX4 Autopilot](https://px4.io)
  * [Micro XRCE-DDS](https://micro.ros.org/docs/concepts/middleware/Micro_XRCE-DDS/)
  * [px4_msgs](https://github.com/PX4/px4_msgs)
  * [QGroundControl](https://qgroundcontrol.com)
  * [Gemini Ultraleap Hand Tracking Software](https://developer.leapmotion.com/tracking-software-download)
  * [Gemini LeapC API Python Bindings](https://github.com/ultraleap/leapc-python-bindings)
* **Hardware**:
  * [Ultraleap Leap Motion Controller Camera](https://leap2.ultraleap.com/products/leap-motion-controller-2/)
  * PX4 Flight Controller (e.g. [Holybro Pixhawk 6X](https://holybro.com/collections/autopilot-flight-controllers/products/pixhawk-6x)) \*
  * Power Module (e.g. [Holybro PM03D](https://holybro.com/collections/power-modules-pdbs/products/pm03d-power-module)) \*
  * Drone Frame (e.g. F450) \*
  * Motors + ESCs \*
  * Companion Computer with Antenna \*
  * RX + TX \*

\* Optional for simulating prototype locally


### Installation

1. **[Ubuntu 22.04](https://ubuntu.com/download/alternative-downloads) System Update:**
``` bash
sudo apt update && apt full-upgrade -y
sudo reboot
```

2. **Install ROS2 Humble:**

Follow the steps [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

3. **Install Python 3.10:**
``` bash
sudo apt install software-properties-common -y
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.10 python3.10-venv python3.10-dev
```

4. **Create and activate a Python virtual environment:**
``` bash

```

5. **Install Gemini LeapC API:**
``` bash
cd ~
git clone https://github.com/ultraleap/leapc-python-bindings.git
cd leapc-python-bindings
pip install -r requirements.txt
python -m build leapc-cffi
pip install leapc-cffi/dist/leapc_cffi-0.0.1.tar.gz
pip install -e leapc-python-api
```

6. **Install PX4 Autopilot:**
``` bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b release/1.14
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

Restart your computer before continuing.

7. **Install PX4 Python Dependencies:**

Install Python dependencies as mentioned in the [PX4 Docs](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2) with this command:

``` bash
pip3 install --user -U empy pyros-genmsg setuptools
pip3 install kconfiglib
pip install --user jsonschema
pip install --user jinja2
```

8. **Build Micro XRCE-DDS:**
``` bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

9. **Setup Workspace:**
``` bash
mkdir -p ~/hand_controlled_drone_ws/src
cd ~/hand_controlled_drone_ws/src
```

10.  **Clone in Packages:**
``` bash
git clone https://github.com/PX4/px4_msgs.git -b release/1.14
```
``` bash
git clone https://github.com/yuriachermann/Hand-Controlled-PX4-Drone.git
```

11.  **Building the Workspace:**
``` bash
source /opt/ros/humble/setup.bash
cd ..
colcon build
```
``` bash
source install/setup.bash
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage

Launch the simulation with the following command:
``` bash
ros2 launch px4_offboard hand_gesture_control.launch.py
```
This will open the Gazebo simulator, RVIZ, and start the necessary ROS2 nodes. Once the system is initialized, you can use hand gestures to control the drone's movement in the simulation.

The controls are as follows:
* Move Hand Up:       Throttle Up
* Move Hand Down:     Throttle Down
* Turn Hand CCW:      Yaw Left
* Turn Hand CW:       Yaw Right
* Turn Hand Forward:  Pitch Forward
* Turn Hand Backward: Pitch Backward
* Turn Hand Left:     Roll Left
* Turn Hand Left:     Roll Right

![Demo](https://i.imgur.com/lTgzHUd.gif)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ROADMAP -->
<!-- ## Roadmap

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->

<!-- CONTACT -->
## Contact

[Yuri Winche Achermann](https://www.linkedin.com/in/yuriachermann/) - yuri.achermann@gmail.com

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

<!-- Specially: -->
* [Jaeyoung Lim](https://github.com/Jaeyoung-Lim) - [px4-offboard](https://github.com/Jaeyoung-Lim/px4-offboard)
* [Braden Wagstaff](https://github.com/bradenwagstaff) - [ROS2_PX4_Offboard_Example](https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example)
* [Harry Mills](https://github.com/HarryMills-UL) - [leapc-python-bindings](https://github.com/ultraleap/leapc-python-bindings)

<!-- Also:
* [Chanjoon Park](https://github.com/ChanJoon)
* [Kasper Grøntved](https://github.com/kasperg3)
* [Huiyu Leong](https://github.com/huiyulhy)
* [Piotr Rosiak](https://github.com/rosiakpiotr)
* [Nikhil S](https://github.com/nikhilsnayak)
* [Alex Klimaj](https://github.com/AlexKlimaj)
* [Bonolo Mathibela](https://github.com/idorobotics)
* [Jacob Dahl](https://github.com/dakejahl)
* [Bonolo Mathibela](https://github.com/idorobotics) -->

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[product-screenshot]: images/screenshot.png
[Python]: https://img.shields.io/badge/Python-000000?style=for-the-badge&logo=python
[Python-url]: https://www.python.org
[Ubuntu]: https://img.shields.io/badge/Ubuntu-000000?style=for-the-badge&logo=ubuntu
[Ubuntu-url]: https://ubuntu.com
[ROS2]: https://img.shields.io/badge/ROS2-000000?style=for-the-badge&logo=ros
[ROS2-url]: https://www.ros.org
[PX4]: https://img.shields.io/badge/PX4-000000?style=for-the-badge&logo=PX4
[PX4-url]: https://px4.io
[Gazebo]: https://img.shields.io/badge/Gazebo-000000?style=for-the-badge&logo=Gazebo
[Gazebo-url]: https://gazebosim.org
[QGroundControl]: https://img.shields.io/badge/QGroundControl-000000?style=for-the-badge&logo=QGroundControl
[QGroundControl-url]: https://qgroundcontrol.com
[Ultraleap]: https://img.shields.io/badge/Ultraleap-000000?style=for-the-badge&logo=Ultraleap
[Ultraleap-url]: https://www.ultraleap.com