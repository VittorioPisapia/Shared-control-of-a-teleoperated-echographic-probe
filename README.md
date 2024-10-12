# Shared control of a teleoperated echographic probe

## Introduction
This work focuses on developing a shared control system for a teleoperated echographic probe integrated with the Franka Emika Panda Robot. To achieve this, we will derive an impedance control law that meets the compliance requirements for the specified tasks. We will then implement a shared control architecture to enhance the collaboration between the manipulator and the human operator. Additionally, we will design several trajectories and approaches for the scanning task, ensuring it can be performed safely and autonomously. This study aims to demonstrate the advantages of shared control in critical scenarios, where effective collaboration between human and robot is essential for precise task execution and the prevention of hazardous situations.

## Control Modalities

 <img src="https://github.com/VittorioPisapia/Shared-control-of-a-teleoperated-echographic-probe/blob/main/images-videos/Robot_with_RF.png" alt="Example Image" style="width:300px;"/>

<div style="text-align: center;">
    <img src="[URL_to_your_image](https://github.com/VittorioPisapia/Shared-control-of-a-teleoperated-echographic-probe/blob/main/images-videos/Robot_with_RF.png)" alt="description" width="300" height="200" />
</div>

To ensure a safe and accurate scan, the task vector is defined as a 6-component vector. The first three 
components represent the x, y, and z cartesian coordinates of the end-effector, which are determined using
the robot’s direct kinematics. The fourth element corresponds to the z-coordinate (expressed in the world
frame) of the fourth joint: by adjusting its height, it is possible to avoid collisions with the patient and
prevent potentially hazardous configurations during the scan. This component is computed using partial
7
direct kinematics. The final two elements describe the orientation, defined using ZYZ Euler angles around
the Z (phi) and Y (theta) axes. The angles have been defined by equating the rotation matrix derived
from the robot’s direct kinematics from the robot’s base to the end-effector, with the composition of three
elementary rotations around the moving ZYZ axes

