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

  public String getCameraName() {
    return cam.getCameraName();
  }

  public Pose3d getEstimatedRobotPose3D() { //? Or change the return type to Optional<Pose3d> and return Optional.empty() instead of null
    return cam.getEstimatedRobotPose3D().orElse(null);
  }

  public Pose2d getEstimatedRobotPose2D() { //? Or change the return type to Optional<Pose3d> and return Optional.empty() instead of null
    return cam.getEstimatedRobotPose2D().orElse(null);
  }

  public Double getLatency() { //? Or change the return type to Optional<Pose3d> and return Optional.empty() instead of null
    return cam.getLatency().orElse(null);
  }
}
