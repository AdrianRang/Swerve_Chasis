# Pure Swerve Chassis
Code for making a swerve chassis move

## What is Swerve Drive in FRC
Swerve drive is a holonomic driving system. Meaning it can move in x and y direction and turn at the same time. You can see an example [here](https://youtu.be/FLnUZBHBczM?si=fxpICCj0WZetGUga&t=17).

Swerve drive works with four wheels called modules, each module has direction and speed, depending on these values the robot can move with the desired speed and direction.

![image](Images/SwervePositionExamples.png)


## How to program swerve

Let's separate swerve drive into the key steps we need to take.

> [!NOTE]
> Swerve drive is ussualy programmed in a field relative reference frame, meaning forward will be forward no matter the rotation of the robot.

We start with the x speed, y speed and rotational speed given as input. We convert to field relative speeds using the gyroscope to get the heading and offset the speeds with that.

Using the field relative speeds we calculate the rotation and speed (Swerve Module State) of each module. 

And finally we need to set the mosules to their corresponding state

> [!IMPORTANT]
> WPILib does all this using the class `ChassisSpeeds`

![diagram](Images/SwerveDriveDiagram.png)
<sup>Image retrieved from [FRC 0 to autnomous](https://youtu.be/0Xi9yb1IMyA?si=rVmkGVnW3SoixsAd)</sup>


### Let's get to the actual code

As we need to factor this into a Command Based Robot we will need to separate Subsystems and commands.

The first subsystem is [`SwerveModule`](src/main/java/frc/robot/subsystems/SwerveModule.java) in which the state of an individual module will be set.

The second one is [`SwerveDrive`](src/main/java/frc/robot/subsystems/SwerveDrive.java) that will be responsible for calculating the state of all modules

And the last one is [`Gyro`](src/main/java/frc/robot/subsystems/Gyro/GyroIOPigeon.java) that will give the rotation of the robot to `SwerveDrive` so it can calculate the field relative speed.

> [!IMPORTANT]
> We use an interface in case we need to switch to a navX instead of our Pigeon 2.0

And we only need one command, [`DriveSwerve`](src/main/java/frc/robot/commands/swerve/DriveSwerve.java) that will send the `xSpeed`, `ySpeed`, and `rotSpeed` to the `SwerveDrive` subsystem

> [!NOTE]
> [`ZeroHeading`](src/main/java/frc/robot/commands/swerve/ZeroHeading.java) is another command but it is only used to set the front of the robot, (the '0' value of the gyroscope)

---

#### [`SwerveModule`](src/main/java/frc/robot/subsystems/SwerveModule.java)

The `SwerveModule` subsystem is initialized with a `SwerveModuleOptions`, this is a class of our oen making and is made to help by making Constants easier to read.

[`SwerveModuleOptions`](src/main/java/lib/team3526/constants/SwerveModuleOptions.java) has 7 variables:

- `absoluteEncoderInverted` ***boolean*** whether the absolute encoder is inverted
- `driveMotorInverted` ***boolean*** whether the drive motor is inverted
- `driveMotorInverted` ***boolean*** whether the turning motor is inverted
- `driveMotorID` ***int*** the CAN ID of the drive motor
- `turningMotorID` ***int*** the CAN ID of the turn motor
- `name` ***String*** the name of the module *ex: 'front left'*

> [!IMPORTANT]
> The turning motor encoder is set to the absolute oncoder's value on intialization, this is because that encoder is not absolute and will always be 0 at start up so you need help from the absolute encoder to know its real position

On initialization (line 46) we set all the variables and reset the encoders. 

`setTargetState()` will [optimize](#Optimizing-a-state) the state and set the speed and angle of the module. the angle is set using a `SparkMaxPIDController`

##### Optimizing a state

We obtimize so the wheel takes the shortest path possible to the desired angle

> 0 degrees with speed of 1 is equal to 180 degrees with speed of -1

![example](Images/Optimization.png)

### [`SwerveDrive`](src/main/java/frc/robot/subsystems/SwerveDrive.java)