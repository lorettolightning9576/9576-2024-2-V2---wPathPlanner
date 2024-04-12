package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import java.io.IOException;
import java.util.Date;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{
    AprilTagFieldLayout fieldLayout;

    static Optional<Alliance> color;

    private final Packet packet = new Packet(1);

    static Double camHeight = Units.inchesToMeters(7.6134);
    static Double angCamToObject = 30.0;
    static Double angCamToAprilTags = 15.0; // degrees facing up

    static Double AprilTagCam_Front_Offset = 0.3048;
    Double AprilTagCam_X_Offset = 0.0;
    Double NoteDetectCam_X_Offset = 0.0;
    static Double Shooter_Aim_Offset = 3.0;

    boolean DoIHaveSpeakerTarget = false;

    Double camYawToSpeaker = null;
    Double camDistToSpeakerTag = null;

    Pose2d lastPose = null;

    public VisionSubsystem() {
        var robotToCamera = new Transform3d(
            new Translation3d(0.0, 0.0, 0.0),
            new Rotation3d(0.0, degreesToRadians(0.0), degreesToRadians(0.0))
        );

        var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        var photonPoseEstimator = new PhotonPoseEstimator(
            layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            new PhotonCamera("camera"),
            robotToCamera
        );
    }

    @Override
    public void periodic() {
        
    }

    /**public PhotonPipelineResult getLatestResult(int cameraIndex) {
        packet.clear();
        var result = new PhotonPipelineResult();
        packet.setData();
    }*/

}
