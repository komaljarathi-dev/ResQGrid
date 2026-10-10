
# ResQGrid — Smart Ambulance Green Corridor System

## Overview

ResQGrid is a smart ambulance green corridor project designed to help an ambulance receive priority at road intersections while travelling toward a hospital.

The prototype uses a Raspberry Pi running QNX, ESP8266 microcontrollers, GPS location data, and TCP/IP communication.

## Objectives

- Receive ambulance location and emergency priority information.
- Determine the nearest intersection and the next hop toward the hospital.
- Communicate priority requests from the QNX-based controller to an intersection controller.
- Provide a mechanism for releasing priority safely after the ambulance passes.

## System Architecture

### QNX Controller
- Platform: Raspberry Pi 4 Model B
- Operating system: QNX
- Role: Process ambulance requests, determine routing decisions, and communicate with the intersection controller.

### Ambulance Unit
- Microcontroller: ESP8266
- Location source: GPS module
- Role: Send ambulance location and emergency-level information to the QNX controller.

### Intersection Controller
- Microcontroller: ESP8266
- Role: Receive priority commands and control the prototype traffic signal.

## Communication

- TCP port 5000: Ambulance request server on the QNX controller.
- TCP port 5001: Intersection controller communication.

## Prototype Road Network

The prototype uses four intersections:

- I1 connects to I2 and I3.
- I2 connects to I1 and I4.
- I3 connects to I1 and I4.
- I4 represents the hospital destination.

## Hardware

- Raspberry Pi 4 Model B
- ESP8266 development boards
- GPS module
- 3×3 matrix keypad, if used in the current prototype
- LEDs or traffic-signal demonstration hardware
- Suitable power supplies and connecting wires

## Software

- QNX Momentics IDE and QNX toolchain
- Arduino IDE
- Git and GitHub
- TCP/IP communication

## Repository Structure

```text
ResQGrid/
├── qnx/
│   └── recoveryehack/
│       ├── Makefile
│       └── src/
│           └── recoveryehack.c
├── esp8266/
├── images/
├── .gitignore
└── README.md
```

## Safety Note

This repository contains a prototype intended for development and demonstration. It is not a certified traffic-control or emergency-response system. Real-world deployment would require validated signal interlocks, fail-safe behavior, reliable ambulance detection, and authorization from relevant traffic authorities.

## Current Status

QNX source code and the Makefile have been uploaded to GitHub. Additional firmware, images, and implementation documentation can be added as they are reviewed and prepared.
## System Architecture Diagram

![ResQGrid Real-Time Ambulance Priority Traffic Architecture](images/Real-Time%20Ambulance%20Priority%20Traffic%20Architecture.png)