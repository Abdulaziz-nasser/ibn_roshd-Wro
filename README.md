# WRO Future Engineers – Autonomous Vehicle

## Team Information

**Team Name:** [Ibn_roshd]

**Country:** [Saudi Arabia, Riyadh]

**Competition:** WRO Future Engineers

**Team Members:**

* [Abdulaziz nasser almindil]

**Coach:** [**Engineer Mohammed Emam**]
[put pics about the team]

We are developing an autonomous car for the WRO Future Engineers competition. Our goal is to build a reliable car that can understand the track, make driving decisions, and complete the challenge autonomously.

## Vehicle

Our car uses an **NVIDIA Jetson Orin Nano** as the main computer.

The Jetson is responsible for:

* Camera processing
* Computer vision
* Detecting track features and obstacles
* Mission decisions
* Logging and debugging

A separate **ESP32-compatible controller** is used for real-time hardware control, including:

* Drive motor
* Steering
* Encoder
* IMU
* Ultrasonic sensors
* Start and reset controls

The Jetson and controller communicate through **USB serial**.

The car is being built and tested step by step so that each part works reliably before we combine everything into the full autonomous system.

## Vehicle Photos

[finish the car]

## Performance Video

[finish the car coding]

A full performance video of the car will be added here as testing progresses.

**Video:** 

More testing videos and results can also be found in the `testing/` folder.


## Open Challenge — 30 Points

Our vehicle is designed to autonomously complete **3 laps**, adapt to the track layout, stay within the course, and stop correctly after finishing.

##Obstacle Challenge — 62 Points

Our vehicle must complete **3 laps** while detecting traffic pillars. It passes **red pillars on the right** and **green pillars on the left**, avoids moving them, and performs **parallel parking** after completing the laps.

