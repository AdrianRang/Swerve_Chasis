// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.LimelightHelpers;

public class LimelightIO implements CameraIO {
  private final String limelightName;
  private Pose3d latestEstimatPose3d;

  public LimelightIO(String limelightName) {
    this.limelightName = limelightName;
  }

  public void periodic() {
    latestEstimatPose3d = LimelightHelpers.getBotPose3d(limelightName);
  }

  public String getCameraName() {
    return limelightName;
  }

  public Optional<Pose3d> getEstimatedRobotPose3D() {
    // TODO: Check if the pose is valid
    return Optional.of(latestEstimatPose3d);
  }

  public Optional<Pose2d> getEstimatedRobotPose2D() {
    // TODO: Check if the pose is valid
    return Optional.of(latestEstimatPose3d.toPose2d());
  }

  public Optional<Double> getLatency() {
    return Optional.ofNullable(LimelightHelpers.getLatency_Capture(limelightName));
  }
}
