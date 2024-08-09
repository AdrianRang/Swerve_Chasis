// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Vision.Camera;

public class PoseEstimator extends SubsystemBase {
  private final SwerveDrive swerve;
  private final Gyro gyro;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Camera[] cameras;

  /** Creates a new PoseEstimator. */
  public PoseEstimator(SwerveDrive swerve, Gyro gyro, Camera[] cameras) {
    this.swerve = swerve;
    this.gyro = gyro;
    this.cameras = cameras;

    poseEstimator = new SwerveDrivePoseEstimator(
      Constants.SwerveDrive.PhysicalModel.kDriveKinematics,
      gyro.getHeading(),
      swerve.getModulePositions(),
      Constants.Field.initialPose,
      //  TODO: Tune these values
      //* These values are optional
      VecBuilder.fill(0.1, 0.1, 0.1),
      VecBuilder.fill(0.05, 0.05, 0.05)
    );
  }

  @Override
  public void periodic() {
    poseEstimator.update(
      gyro.getHeading(),
      swerve.getModulePositions()
    );
    
    for (Camera camera : cameras) {
      if(camera.getEstimatedRobotPose2D() == null) continue;
      //? Or if(camera.getEstimatedRobotPose2D().isEmpty()) continue;

      poseEstimator.addVisionMeasurement(
        camera.getEstimatedRobotPose2D(),
        camera.getLatency()
      );
    }
  }
}
