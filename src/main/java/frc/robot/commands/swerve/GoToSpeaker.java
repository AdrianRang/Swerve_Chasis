// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.subsystems.SwerveDrive;;

public class GoToSpeaker extends Command {
  private final SwerveDrive swerve;
  private Command command;
  private final PathConstraints constraints;
  /** Creates a new GoToSpeaker. */
  public GoToSpeaker(SwerveDrive swerve) {
    this.swerve = swerve;

    constraints = new PathConstraints(
      Constants.Pathfinding.kMaxSpeed.in(MetersPerSecond),
      Constants.Pathfinding.kMaxAcceleration.in(MetersPerSecondPerSecond),
      Constants.Pathfinding.kMaxAngularSpeed.in(RadiansPerSecond),
      Constants.Pathfinding.kMaxAngularAccelerationRPS2
    );

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // https://github.com/TitaniumTitans/2024Crescendo/blob/526e6f69abb8501e16003e18f45b19bf95297557/src/main/java/frc/robot/commands/AlignmentDriveCommand.java#L57
    command = new PathfindHolonomic(
      Constants.Pathfinding.SpeakerPose,
      constraints,
      0.0,
      swerve::getPose,
      swerve::geRelativeChassisSpeeds,
      swerve::driveRobotRelative,
      Constants.Pathfinding.config,
      0.0,
      swerve
    );

    command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return command.isFinished();
  }
}
