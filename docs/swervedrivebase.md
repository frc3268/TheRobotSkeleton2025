# Swerve Subsystem

**As mentioned in the links on the Intro page, every mechanism on a robot is its own subsystem.** That includes the chassis/drivebase. The major components of a Swerve Chassis are as follows:
- Swerve Modules(x4): Control and track the movement of the wheels on the chassis
- Gyroscope: Senses the pitch, roll, and yaw of the chassis.

In essence, code for a Swerve Chassis needs to do the following:
- Control the movement of the chassis, provide interfaces for such(so that other parts of the code can control the movement of the chassis)
- Report the position and state of the Chassis
- Interface with the Gyroscope

The code for the Swerve Chassis is stored in the `src/main/kotlin/frc/lib/basics/SwerveDriveBase.kt` file of the ValhallaLib repo. 

_But what does each part do?_