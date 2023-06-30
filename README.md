# Thrust-Vectoring Rocket

This repository contains software for controlling a 
thrust-vectoring model rocket.

I am using a Teensy 4.1 on a custom PCB christened 
the OTTAr1.3(On-board Telemetry and TVC Avionics),
and I am working out of PlatformIO in vscode.

# Contents

1. Main .cpp code containing a program to control 
a two-servo thrust vector mount
2. Several subclasses for larger functions, as well 
as to utilize OOP
    - The "pid" subclass contains the control loop: 
    a Proportional, Integral, Differential feedback
    loop provides a predictive and memory-based
    controller for the thrust gimbaling.
3. All libraries and config files required to run 
out of PIO instead of arduino framework

# Installation

Install PIO in vscode, then open repo through "open
project" in PIO