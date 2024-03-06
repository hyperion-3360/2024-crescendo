Neo 550 and Neo motors are CANSparkMax in code
CODING STANDARDS:
1. all constants start with a k (not capital)
2. all members (ex motors, sensors, encoders, etc)
3. names are self documented (ex m_climberRight, ShootNote(), kLeftFrontDriveId)
4. if names or methods are unclear, add doc with /* doc */

april tag coordinates:

ID: 1 X: 593.68, Y: 9.68, Z: 53.38, ROTATION: 120°

ID: 2 X: 637.21, Y: 34.79, Z: 53.38, ROTATION:120°

ID: 3 X: 652.73, Y: 196.17, Z: 57.13, ROTATION:180°

ID: 4 X: 652.73, Y: 218.42, Z: 57.13, ROTATION:180°

ID: 5 X: 578.77, Y: 323.00, Z: 53.38, ROTATION:270°

ID: 6 X: 72.5, Y: 323.00, Z: 53.38, ROTATION:270°

ID: 7 X: -1.50, Y: 218.42, Z: 57.13, ROTATION:0°

ID: 8 X: -1.50, Y: 196.17, Z: 57.13, ROTATION:0°

ID: 9 X: 14.02, Y: 34.79, Z: 53.38, ROTATION:60°

ID: 10 X: 57.54, Y: 9.68 ,Z: 53.38, ROTATION:60°

ID: 11 X: 468.69, Y: 146.19, Z: 52.00, ROTATION:300°

ID: 12 X: 468.69, Y: 177.10, Z: 52.00, ROTATION:60°

ID: 13 X: 441.74, Y: 161.62, Z: 52.00, ROTATION:180°

ID: 14 X: 209.48, Y: 161.62, Z: 52.00, ROTATION:0°

ID: 15 X: 182.73, Y: 177.10, Z: 52.00, ROTATION:120°

ID: 16 X: 182.73, Y: 146.19, Z: 52.00, ROTATION:240°
>>>>>>> odometry

>> AUTO INFO <<

4NotesMidField works, does it fit time?
4NotesFarShot could replace previous, to test

center field autos need modifs so that far shot is possible

# Shooting in motion 
The whole idea behind this feature is to give the pilot the ability to keep the aim using odometry and Apriltag detection while moving the robot to avoid obstacles.

## Behavior description
Once engaged by the pilot, and until explicitely released, the code will take the control or Pose of the robot wrt the target and the elevator angle will be modified to maintain an optimal shooting position. It is assumed the shooting speed will remain constant.  The LED subsystem will indicate the readiness of the system to shoot the note. The aim lock mechanism will remain in function even when the distance between the robot and the target is greater than what can be reached by the shooter. In the latter case, the LED subsystem will notify the pilot (alongside with a shoot ready display in shuffleboard) of that status.

The aim locking mechanism will be engaged by the pilot controller right trigger and will remain in action as long as the trigger is pressed.

The LED subsystem will indicate optimal shooting condition using steady BLUE illumination and the copilot controller will vibrate

The LED subsystem will indicate an unreachable target (or not able to lock) using the FAST FLASH WHITE and the copilot controller will stop vibrate (assuming it was aim lock before)

## Automatic target selection
The AprilTag detection system will be used to automatically select the target to lock on. The alliance will also be considered to make sure we score in the appropriate module! In the unlikely even the vision system detects AprilTag of both the Amplifier and the Speaker, the priority will be given to the Speaker. Once locked on a target will remain valid until another target is visible by the vision system or until the system is disengaged.

## Expectations wrt odometry precsion
This aim lock command is using odometry to determine the distance between the robot and the target and calculate the angle of the elevator. It is also using the Pose2D to determine the angle between the robot and the target. Given the implicitely imprecise nature of odometry using a gyroscope, it is expected machine vision with AprilTag detection will vastly improve odometry accuracy.

As long as an Apriltag is maintained in the camera field of view the odometry is corrected in real time providing the best possible angle and position accuracy. However as soon as the Apriltag is not detected, the regular (gyro based) odometry is used transparently by the aim locking command.