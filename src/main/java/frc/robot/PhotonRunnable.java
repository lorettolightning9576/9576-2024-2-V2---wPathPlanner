package frc.robot;

import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT;
import static frc.robot.Constants.VisionConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH_METERS;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;

public class PhotonRunnable implements Runnable{
    
    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera photonCamera;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

    public PhotonRunnable() {
        this.photonCamera = new PhotonCamera("photonCamera");
        PhotonPoseEstimator photonPoseEstimator = null;

        var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        if(photonCamera != null) {
            photonPoseEstimator = new PhotonPoseEstimator(
                layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, APRILTAG_CAMERA_TO_ROBOT.inverse()
            );
        }
        this.photonPoseEstimator = photonPoseEstimator;
    }

    @Override
    public void run() {
        if (photonPoseEstimator != null && photonCamera != null && !RobotState.isAutonomous()) {
            var photonResult = photonCamera.getLatestResult();
            if (photonResult.hasTargets()
                && (photonResult.targets.size() > 1 || photonResult.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
                photonPoseEstimator.update(photonResult).ifPresent(estimatedRobotPose -> {
                    var estimatedPose = estimatedRobotPose.estimatedPose;
                    if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
                        && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
                        atomicEstimatedRobotPose.set(estimatedRobotPose);
                    }
                });
            }
        }
    }

    /**
   * Gets the latest robot pose. Calling this will only return the pose once. If it returns a non-null value, it is a
   * new estimate that hasn't been returned before.
   * This pose will always be for the BLUE alliance. It must be flipped if the current alliance is RED.
   * @return latest estimated pose
   */
    public EstimatedRobotPose grabLatestEstimatedPose() {
        return atomicEstimatedRobotPose.getAndSet(null);
    }
}
