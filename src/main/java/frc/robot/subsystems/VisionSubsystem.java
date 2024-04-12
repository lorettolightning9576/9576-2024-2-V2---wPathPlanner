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
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.proto.PhotonPipelineResultProto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

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

    Double lastFPGATime = null;

    Double camYawToSpeaker = null;
    Double camDistToSpeakerTag = null;

    Pose2d lastPose = null;

    public static PhotonPipelineResult lastResult;

    Double lastTimestamp = null;

    // Set Target Speaker Positions
    public static Translation2d blueSpeakerPos = new Translation2d(0.076, 5.547868);
    public static Translation2d redSpeakerPos = new Translation2d(16.465042, 5.547868);

    public static final double targetWidth = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters

    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 197
    public static final double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters

    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    // pages 4 and 5
    public static final double kFarTgtXPos = Units.feetToMeters(54);
    public static final double kFarTgtYPos = Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75)- Units.inchesToMeters(48.0 / 2.0);
    
    public static final double kFarTgtZPos = (Units.inchesToMeters(98.19) - targetHeight) / 2 + targetHeight;

    public static final Pose3d kFarTargetPose = new Pose3d(
            new Translation3d(kFarTgtXPos, kFarTgtYPos, kFarTgtZPos),
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(180))
    );

    public static Transform3d robotToCamera = new Transform3d(
            new Translation3d(0.0, 0.0, 0.0),
            new Rotation3d(0.0, degreesToRadians(0.0), degreesToRadians(0.0))
    );

    public static PhotonCamera camera;

    public VisionSubsystem() {

        SmartDashboard.putNumber("rotation", 0);

        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            fieldLayout = null;
        }

        camera = new PhotonCamera("camera");
    }

    @Override
    public void periodic() {
        var photonPoseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            camera,
            robotToCamera
        );

        PhotonPipelineResult results = camera.getLatestResult();
        lastResult = results;
        
        /**if (lastTimestamp == null || lastTimestamp != results.getTimestampSeconds()) {
            lastTimestamp = results.getTimestampSeconds();
            lastFPGATime = results.getTimestampSeconds();
        } else {
            ;
        }*/

        color = DriverStation.getAlliance();

        if (color.isEmpty()) return;

        Pose2d currentPose = RobotContainer.drivebase.getPose();

        if (color.get() == Alliance.Blue) {
            camDistToSpeakerTag = currentPose.getTranslation().getDistance(blueSpeakerPos) - AprilTagCam_Front_Offset;
            camYawToSpeaker = normalizeAngle(currentPose.getTranslation().minus(blueSpeakerPos).getAngle().getDegrees() + Shooter_Aim_Offset);
        } else {
            camDistToSpeakerTag = currentPose.getTranslation().getDistance(redSpeakerPos) - AprilTagCam_Front_Offset;
            camYawToSpeaker = normalizeAngle(currentPose.getTranslation().minus(redSpeakerPos).getAngle().getDegrees() + Shooter_Aim_Offset);
        }

        SmartDashboard.putNumber("Vision.DistToSpeakerTag", Units.metersToFeet(camDistToSpeakerTag));
        SmartDashboard.putNumber("Vision.camYawToSpeaker", camYawToSpeaker);
    }

    public Double getCamYawToSpeaker() {
        return camYawToSpeaker;
    }

    public Double getCamDistToSpeaker() {
        return camDistToSpeakerTag;
    }

    /**public static PhotonPipelineResult findtargetInResults(PhotonPipelineResult photonResult, int id) {
        for (var target : photonResult.targets.) {
            if (target.doubleValue() == id) {
                return target.doubleValue();
            }
        }
        return null;
                var tarID = photonResult.getTargets().get(id);
    }*/

    /**public static PhotonPipelineResult findtargetIsnResults(PhotonPipelineResult photonResult, int id) {
        for (var tarID : photonResult.getMultiTagResult().fiducialIDsUsed) {
            if (tarID == id) {
                return tarID;
            }
        }
        return null;
    }*/

    /**public PhotonPipelineResult getLatestResult(int cameraIndex) {
        packet.clear();
        var result = new PhotonPipelineResult();
        packet.setData();
    }*/

      /**
    * This method makes sure the angle difference calculated falls between -180
    * degrees and 180 degrees
    * 
    * @param angle
    * @return
    */
    public static double normalizeAngle(double angle) {
        angle = angle % 360;

        if (angle > 180) {
            angle = -360 + angle;
        }
        if (angle <= -180) {
            angle = 360 + angle;
        }

        if (angle == -0) {
            angle = 0;
        }

        return angle;
    }

}
