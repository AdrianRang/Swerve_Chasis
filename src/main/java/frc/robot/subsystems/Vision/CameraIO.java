package frc.robot.subsystems.Vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface CameraIO {
    public String GetCameraName();

    public default void periodic() {};

    Optional<Pose3d> GetEstimatedRobotPose3D();
    Optional<Pose2d> GetEstimatedRobotPose2D();
    Optional<Double> GetLatency();
}
