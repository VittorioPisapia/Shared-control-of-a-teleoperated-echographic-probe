# Shared control of a teleoperated echographic probe

<p align="center">
    <img src="https://github.com/VittorioPisapia/Shared-control-of-a-teleoperated-echographic-probe/blob/main/images-videos/Free2.gif" alt="Example Image" style="width:660px;"/>
</p>


## Introduction
This work focuses on developing a shared control system for a teleoperated echographic probe integrated with the Franka Emika Panda Robot. To achieve this, we will derive an impedance control law that meets the compliance requirements for the specified tasks. We will then implement a shared control architecture to enhance the collaboration between the manipulator and the human operator. Additionally, we will design several trajectories and approaches for the scanning task, ensuring it can be performed safely and autonomously. This study aims to demonstrate the advantages of shared control in critical scenarios, where effective collaboration between human and robot is essential for precise task execution and the prevention of hazardous situations.

## Task definition

The task vector is defined as a 6-component vector. The first three components represent the x, y, and z cartesian coordinates of the end-effector, which are determined using the robot’s direct kinematics. The fourth element corresponds to the z-coordinate (expressed in the world frame) of the fourth joint: by adjusting its height, it is possible to avoid collisions with the patient and prevent potentially hazardous configurations during the scan. The final two elements describe the orientation, defined using ZYZ Euler angles around the Z (phi) and Y (theta) axes.


## Control modalities

The position and orientation of the robot’s end-effector can be modified in real time using external devices, such as an Xinput-compatible controller. This enables the operator to precisely adjust the echographic probe during the scan for more detailed analysis.

### Shared control

Although the operator can manually adjust the end-effector’s position, several safety precautions have been implemented to ensure patient protection. Specifically, a shared control protocol has been introduced by setting a safety threshold: if the force sensor detects a force along the Z-axis exceeding this threshold, the control system prevents any further increase in force in that direction.

## Trajectories

### Echo Mode

<p align="center">
    <img src="https://github.com/VittorioPisapia/Shared-control-of-a-teleoperated-echographic-probe/blob/main/images-videos/echo.gif" alt="Example Image" style="width:660px;"/>
</p>

### Linear Trajectory

<p align="center">
    <img src="https://github.com/VittorioPisapia/Shared-control-of-a-teleoperated-echographic-probe/blob/main/images-videos/linear.gif" alt="Example Image" style="width:660px;"/>
</p>

### Wrist Trajectory

<p align="center">
    <img src="https://github.com/VittorioPisapia/Shared-control-of-a-teleoperated-echographic-probe/blob/main/images-videos/wrist.gif" alt="Example Image" style="width:660px;"/>
</p>



