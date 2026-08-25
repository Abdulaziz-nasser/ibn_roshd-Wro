# ibn_roshd-Wro
# WRO Future Engineers – Autonomous Vehicle

## Team Information

**Team Name:** [Ibn_roshd]
**Country:** [Saudi Arabia, Riyadh]
**Competition:** WRO Future Engineers
**Team Members:**

* [Abdulaziz nasser almindil]

**Coach:** [**Engineer Mohammed Emam**]

We are developing an autonomous vehicle for the WRO Future Engineers competition. Our goal is to build a reliable vehicle that can understand the track, make driving decisions, and complete the challenge autonomously.

## Vehicle

Our vehicle uses an **NVIDIA Jetson Orin Nano Super** as the main computer.

The Jetson is responsible for:

* Camera processing
* Computer vision
* Detecting track features and obstacles
* Mission decisions
* Logging and debugging

A separate **Arduino-compatible controller** is used for real-time hardware control, including:

* Drive motor
* Steering
* Encoder
* IMU
* Ultrasonic sensors
* Start and reset controls

The Jetson and controller communicate through **USB serial**.

The vehicle is being built and tested step by step so that each part works reliably before we combine everything into the full autonomous system.

### Vehicle Photos



## Performance Video

A full performance video of the vehicle will be added here as testing progresses.

**Video:** 

More testing videos and results can also be found in the `testing/` folder.
