// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  private final CameraIO cam;

  /** Creates a new Camera. */
  public Camera(CameraIO cam) {
    this.cam = cam;
  }

  @Override
  public void periodic() {
    cam.periodic();
  }

  public String GetCameraName() {
    return cam.GetCameraName();
  }

  public Pose3d GetEstimatedRobotPose3D() {
    return cam.GetEstimatedRobotPose3D().orElse(null);
  }

  public Pose2d GetEstimatedRobotPose2D() {
    return cam.GetEstimatedRobotPose2D().orElse(null);
  }

  public Double GetLatency() {
    return cam.GetLatency().orElse(null);
  }
}
