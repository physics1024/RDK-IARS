# RDK - IARS (Intelligent Accompaniment Robot System)

## 1. Project Overview

### 1.1 Functions and Features

**This system is based on the RDK X5 platform and includes the following core functions:**
- **Intelligent Fall Detection**: Implements fall detection through pose recognition algorithms, supporting voice confirmation and remote emergency alerts.
- **Precise Medication Management**: Equipped with a rotary smart pillbox, integrated with cloud-based medication schedule input and voice reminders.
- **AI Health Assistant**: Integrates a Large Language Model (LLM) to provide health Q&A, medical consultation, and voice companionship services.
- **Autonomous Navigation Capability**: Completes independent map building, enabling point-to-point navigation and autonomous environmental patrols.
- **Remote Monitoring Platform**: Allows for remote control, video surveillance, voice interaction, and medication schedule adjustments via a web interface.

The system overcomes the limitations of traditional, stationary companion devices to deliver an active, mobile, and intelligent care solution.

### 1.2 Application Areas

- **Core Scenario**: In-home care for elderly individuals living alone, addressing key needs such as fall emergencies, medication management, and health consultations.
- **Extended Applications**: Child safety monitoring, daily assistance for individuals with disabilities, and rehabilitation companionship.
- **Social Value**: Contributes to building a smart elderly care ecosystem, promotes the industrialization of "AI + Elderly Care," and alleviates family caregiving burdens.

### 1.3 Key Technical Features

- **Multimodal Technology Integration**: Integrates a human pose recognition visual model, a Large Language Model, and speech recognition & synthesis technologies.
- **High-Efficiency Hardware Platform**: Utilizes the RDK X5 platform and the TogetheROS framework to provide efficient on-board computing power.
- **Autonomous Navigation System**: Employs the Cartographer algorithm for environmental mapping and the Navigation2 framework to ensure navigation precision.
- **Remote Interaction System**: Achieves real-time monitoring of the home environment through low-latency video streaming and remote control.
- **Integrated Design**: Builds a complete perception-decision-execution closed-loop system by integrating "Hardware + Algorithms + Software Platform."

### 1.4 Key Performance Indicators

|               Metric               | Parameter |
| :--------------------------------: | :-------: |
|      Fall Detection Accuracy       |   ≥97%    |
| Fall Detection False Positive Rate |    ≤3%    |
|    Human Following Success Rate    |   ≥95%    |
|     Navigation Waypoint Error      |   ≤0.1m   |
|    Pillbox Reminder Time Error     |  ≤500ms   |
|     Voice Interaction Latency      |    ≤1s    |
| Remote Video Transmission Latency  |  ≤400ms   |

### 1.5 Key Innovations

- **Mobile Intelligent Companionship**: Breaks the constraints of fixed devices to achieve autonomous mobile patrols and proactive risk monitoring.
- **Deep AI Integration**: Fuses computer vision, language models, SLAM, and other technologies to form a complete intelligent decision-making system.
- **Affective Interaction Design**: Provides human-like health consultations and daily companionship through speech recognition and synthesis.
- **Edge-Cloud Collaborative Architecture**: Balances efficiency and functional requirements by combining rapid on-board response with in-depth cloud-based processing.
- **Closed-Loop Management Design**: Establishes a full-process service chain covering medication reminders, status monitoring, and family member management.

## 2. Installation and Deployment
### 2.1 Hardware Requirements
- RDK X5 Development Platform
- PTZ Camera (requires custom adaptation)
- Rotary Pillbox Module (custom design)
- USB Microphone and Speaker
- External Network Card (optional)

### 2.2 Software Requirements

The required system is the **official RDK Ubuntu 22.04** image, which includes the official **Together ROS** system and the official **ROS Humble** system.
The system must have the **official ROS Bridge** and **ALSA audio drivers** configured. A full list of dependencies is still being compiled.

### 2.3 Single-Module Deployment

Each folder corresponds to a specific function. To run, navigate to the respective folder and execute `run.sh`.
The `WWW` folder contains the back-end service, which needs to be exposed to the public internet using an intranet penetration tool like [frp](https://github.com/fatedier/frp).
Calling the Alibaba Cloud Bailian Large Language Model requires an API KEY. Please refer to the [Bailian LLM Introduction](https://help.aliyun.com/zh/model-studio/realtime) for deployment.

## 3. Development and Contribution

You are welcome to contribute to the project in the following ways:
1. Submit an Issue to report problems or make suggestions.
2. Fork the repository and submit a Pull Request.
3. Improve the project documentation and test cases.

## 4. License

This project is licensed under the **GNU GPL v3.0** open-source license. For details, please see the LICENSE file.

------------
Intelligent Guardian, Warm Companionship - Bringing Warmth to Every Moment with Technology

© 2025 Tongxin Zhihu Project Team