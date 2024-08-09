package frc.robot.subsystems.Vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class PhotonIO implements CameraIO {
    private final String photonName;
    private Optional<Pose3d> latestEstimatPose3d;
    private final PhotonCamera cam;
    private final Transform3d cameraToRobot;

    public PhotonIO(String photonName, Transform3d cameraToRobot) {
        this.photonName = photonName;
        this.cam = new PhotonCamera(photonName);
        this.cameraToRobot = cameraToRobot;
    }

    public void periodic() {
        var result = cam.getLatestResult();
        if(result.hasTargets()){
            var target = result.getBestTarget();
            if(Constants.Field.aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()){
                latestEstimatPose3d = Optional.of(PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), Constants.Field.aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot));
            }else{
                DriverStation.reportWarning("No AprilTag pose found for target " + target.getFiducialId(), false);
                latestEstimatPose3d = Optional.empty();
            }
        }
    }

    public String getCameraName() {
        return photonName;
    }

    public Optional<Pose3d> getEstimatedRobotPose3D() {
        return latestEstimatPose3d;
    }
    
    public Optional<Pose2d> getEstimatedRobotPose2D() {
        if (latestEstimatPose3d.isEmpty()) return Optional.empty();
        return Optional.of(latestEstimatPose3d.get().toPose2d());
    }

    public Optional<Double> getLatency() {
        return Optional.of(cam.getLatestResult().getLatencyMillis());
    }
}