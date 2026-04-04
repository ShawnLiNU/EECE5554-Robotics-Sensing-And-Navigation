# EECE 5554 — Robotics Sensing and Navigation

<p align="left">
  <strong>Northeastern University · Seattle Campus</strong><br>
  Department of Electrical and Computer Engineering<br>
  Instructor: <a href="https://shawnlinu.github.io">Dr. Xian Li</a>
</p>

---

## Course Overview

EECE 5554 introduces a systems-level view of introductory robotics with emphasis on sensing and navigation methodologies in the context of hardware and software systems — including power, embedded systems, mechanical design, and robotic software architectures. The course involves extensive programming in Python with hands-on use of OpenCV and ROS2.

Students work with state-of-the-art sensors throughout the semester, progressing from raw serial data to complete multi-sensor fusion pipelines.

## What You Will Do

- **Write device drivers** for GPS receivers and IMUs from scratch over USB serial
- **Convert and analyze** GNSS data across coordinate systems (lat/lon, UTM)
- **Characterize sensor noise** using Allan variance and statistical analysis
- **Build inertial navigation** pipelines fusing accelerometer, gyroscope, and magnetometer data
- **Drive around the city** collecting real multi-sensor data and reconstructing your trajectory
- **Stitch panoramic mosaics** from overlapping images using feature detection and homographies
- **Design a socially relevant robot** applying systems-level sensing and navigation principles

## Sensors and Hardware

| Sensor | Used In | Key Data |
|---|---|---|
| GNSS Puck (GPS) | Labs 1, 2, 5 | NMEA sentences (lat/lon/alt) at 1 Hz |
| VectorNav VN-100 IMU | Labs 3, 4, 5 | Accel, gyro, magnetometer at up to 800 Hz |
| Camera (phone) | Lab 6 | Images for feature detection and mosaicing |
| NUance Vehicle | Lab 5 | Autonomous car platform for driving data collection |

## Course Structure

| Phase | Topics | Labs |
|---|---|---|
| **0 — Foundations** | Linux, Git, ROS2 pub-sub, packages, nodes | Lab 0: ROS basics |
| **1 — GNSS** | Map projections, UTM, GPS, RTK, serial comm, signal processing | Lab 1: GNSS drivers · Lab 2: RTK analysis |
| **2 — Inertial** | Allan deviation, 3D transforms, IMU, magnetometer calibration, power systems | Lab 3: Allan variance · Lab 4: Walking odometry · Lab 5: Driving navigation |
| **3 — Vision** | Homographies, features, stereo, calibration, mosaicing | Lab 6: Panoramic mosaic |
| **4 — SLAM** | Bayes filters, Kalman filter, particle filter, ICP, graph SLAM, mission planning | Theory + Individual project |

## Prerequisites

- Intermediate Python (or strong background in another language)
- Basic statistics (mean, std, histograms, normal distributions)
- Linear algebra fundamentals (matrix multiplication, eigenvalues, rotation matrices)
- A laptop capable of running Ubuntu 24.04 (8+ GB RAM, 30+ GB free disk, 2 USB ports)

See the [Getting Started](../../wiki/Getting-Started) wiki page for detailed prep resources.

## Repository Structure

```
EECE5554-Robotics-Sensing-And-Navigation/
├── README.md
├── lab0/                  # ROS2 basics
│   └── ...
├── lab1/                  # GNSS driver, custom messages, rosbags
│   ├── src/
│   │   ├── gps_driver.py
│   │   └── ...
│   ├── msg/
│   │   └── GPS.msg
│   ├── launch/
│   │   └── gps_launch.py
│   └── data/
│       ├── stationary_open.bag
│       ├── stationary_occluded.bag
│       └── walking.bag
├── lab2/                  # RTK GNSS analysis
│   ├── analysis/
│   │   └── rtk_analysis.py
│   └── ...
├── lab3/                  # IMU driver, Allan variance
│   ├── src/
│   │   └── imu_driver.py
│   ├── analysis/
│   │   └── allan_variance.py
│   └── data/
│       └── stationary.bag
├── lab4/                  # Inertial odometry (walking)
│   ├── analysis/
│   │   ├── circle_analysis.py
│   │   └── square_analysis.py
│   └── ...
├── lab5/                  # NUance navigation (driving)
│   ├── launch/
│   │   └── sensors_launch.py
│   ├── analysis/
│   │   ├── magnetometer_calibration.py
│   │   ├── heading_estimation.py
│   │   └── trajectory_estimation.py
│   └── data/
│       ├── data_going_in_circles.bag
│       └── data_driving.bag
├── lab6/                  # Camera mosaicing
│   ├── main.ipynb
│   ├── images/
│   └── ...
└── Individual_Project/    # Robot system design (5 slides)
    └── robot_design.pptx  # or .pdf
```

## Software Stack

| Tool | Version | Purpose |
|---|---|---|
| Ubuntu | 24.04 LTS | Operating system |
| ROS2 | Humble Hawksbill | Robotics middleware |
| Python | 3.10+ | All driver and analysis code |
| OpenCV | 4.x | Computer vision (Lab 6) |
| NumPy / SciPy / Matplotlib | Latest | Data analysis and plotting |
| AllanTools | Latest | Allan variance computation (Lab 3) |
| UTM | Latest | Coordinate conversion (Labs 1–2) |
| Minicom | — | Serial port debugging |

## Wiki

**[Course Wiki](../../wiki)** — Lessons, lab guides, reference material, and troubleshooting.

The wiki contains 21 lessons covering GPS, RTK, IMUs, cameras, LiDAR, and SLAM algorithms, plus detailed guides for all 7 labs and the individual project.

## Acknowledgments

This course uses state-of-the-art sensors, some of which are quite expensive. Please be careful — but if something breaks, contact the TA or instructor so we can order a replacement to keep the class moving smoothly.

Course materials are proprietary to Dr. Xian Li and Northeastern University. Educational use with attribution is welcome.

Special thanks to [Dr. Kris Dorsey](https://coe.northeastern.edu/people/dorsey-kris/) and [Dr. Hanumant Singh](https://coe.northeastern.edu/people/singh-hanumant/) for providing the foundational course materials.

---

<p align="center">
  <em>"No guts, no glory. No fun, no story." - R.G. Nuranen</em>
</p>
