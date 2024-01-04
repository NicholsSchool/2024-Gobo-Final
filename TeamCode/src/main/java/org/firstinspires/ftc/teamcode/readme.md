GOALS/TODOS/NOTES ...

Will assuming the wrist offset at the start of teleop cause problems?

Be mindful with loop times with each new functionality (especially IMU and Vision)
Current Estimate: empty = 1-2 millis, no vision/imu 10-30 millis

Have James put TF file on the robot

Consider re-drilling odometry (and/or tightening front one) if it shows to be a problem

Verified:
Both profiles work as intended (both need tuning for all cases)
Controller works as intended
IMU works as intended with negligible loop delay
Hand works as intended

Testing Order:
Lights (isn't working at all)
Arm
Drivetrain