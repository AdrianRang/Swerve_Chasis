package frc.robot.subsystems.Vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface CameraIO {
    public default void periodic() {};

    public String getCameraName();

    Optional<Pose3d> getEstimatedRobotPose3D();
    Optional<Pose2d> getEstimatedRobotPose2D();
    Optional<Double> getLatency();
}
