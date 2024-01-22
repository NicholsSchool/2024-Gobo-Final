        NOTES ...
AprilTagGameDatabase.java contains the field positions of all tags

        Large TODOS:
Organize and Modify all Constants
Change all Angles to Radians
Automated Plane Launch Routine
Automated Climb Routine (using IMU pitch?)
Make Point class and use for all profiles/pose/joystick inputs

        Small TODOS:
Hand - debug, optimize, and javadoc
Hand - redo Servo ranges to be perfectly 45 degrees, and close well

Vision - debug, optimize, and javadoc
Vision - if we ever go back to using continuously, add motion profiling
Vision - Measure Camera Offset again
Vision - Test Radians change

Arm - debug, optimize, and javadoc
Arm - redo Plane launcher range to be perfect
Arm - tune the climb motion profiling speed and max
Arm - tune shoulder max speed
Arm - tune shoulder goToPosition
Arm - tune new wrist max speed
Arm - tune new wrist goToPosition
Arm - tune virtual fourbar switching angle
Arm - tune new wrist virtual fourbar angles

Drivetrain - debug, optimize, and javadoc
Drivetrain - tune drive motor velocities
Drivetrain - tune the driving profile speed
Drivetrain - tune virtual gearing
Drivetrain - tune autoAlign allowed error for radians
Drivetrain - tune autoAlign p for radians
Drivetrain - tune spline p
Drivetrain - tune spline allowed error
Drivetrain - teleop spline through opposite truss path
Drivetrain - optimize all spline waypoints
Drivetrain - re-tune odometry correction




Robot - remove telemetry method after checking final loop time
Robot - don' check for isBlueAlliance in loop()



ControllerTeleop - color, rotate, and scale the grid. Center the square
AutoRobot - Auto through center is a bad idea for next comp
