# Shared control of a teleoperated echographic probe
## Table of contents
- [Revolutionizing Medical Robotics with Shared Control](#revolutionizing-medical-robotics-with-shared-control)
- [Key Features](#key-features)
- [Control Modes](#control-modes)
- [Trajectory Modes](#trajectory-modes)
- [Why Shared Control](#why-shared-control)
- [How to run the code](#how-to-run-the-code)

## Revolutionizing Medical Robotics with Shared Control
<p align="center">
    <img src="https://github.com/VittorioPisapia/Shared-control-of-a-teleoperated-echographic-probe/blob/main/images-videos/Free2.gif" alt="Example Image" style="width:660px;"/>
</p>

This project showcases the power of **shared control** in a teleoperated echographic probe, integrated with the advanced **Franka Emika Panda Robot**. By leveraging **impedance control laws** and intelligent safety protocols, this system enhances the collaboration between human operators and robotic manipulators—offering a new level of precision and safety in medical imaging.

## Key Features

- **Real-time Control**: Precise positioning and orientation adjustments using an external controller.
- **Intelligent Safety Protocols**: Built-in force-sensing to prevent excessive pressure during scans.
- **Advanced Trajectories**: Autonomous and guided paths for smooth, accurate scanning.

## Control Modes

1. **Manual Control**: Operator adjusts probe position with fine-tuned real-time input.
2. **Trajectories**: Excecutes predefined trajectories in order to avoid risks and ensure safe operation.

## Trajectory Modes

Detailed description for each trajectories can be found in `Report_PDF.pdf` placed in `Presentation` folder.

### Echo Mode
<p align="center"> <img src="https://github.com/VittorioPisapia/Shared-control-of-a-teleoperated-echographic-probe/blob/main/images-videos/echo.gif" alt="Echo Mode" style="width:660px;"/> </p>


### Linear Trajectory
<p align="center"> <img src="https://github.com/VittorioPisapia/Shared-control-of-a-teleoperated-echographic-probe/blob/main/images-videos/linear.gif" alt="Linear Trajectory" style="width:660px;"/> </p>

### Wrist Trajectory
<p align="center"> <img src="https://github.com/VittorioPisapia/Shared-control-of-a-teleoperated-echographic-probe/blob/main/images-videos/wrist.gif" alt="Wrist Trajectory" style="width:660px;"/> </p>

## Why Shared Control?

This approach enables seamless collaboration, combining **human intuition** with **robotic precision**, offering:

- Enhanced safety in delicate medical procedures.
- Greater operator comfort and reduced fatigue.
- More precise outcomes in real-time medical applications.

Join us in shaping the future of robotic healthcare!

## How to run the code
1. Install all the requirements (check `requirements.txt`)
2. Set the CoppeliaSim scene and set as **Dynamic engine** '*Bullet 2.78*'
3. Run `main.m`placed in `MATLAB_code`
   
There is a known bug that occurs when running MATLAB code simultaneously with CoppeliaSim during the initialization of transmission channels for data. The first run will always fail, resulting in an error in CoppeliaSim.

To resolve this issue, you can terminate the simulation in both MATLAB and CoppeliaSim, and then restart the simulation.
