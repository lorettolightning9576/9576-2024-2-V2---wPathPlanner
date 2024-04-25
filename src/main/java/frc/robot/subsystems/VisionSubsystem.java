package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase{
    private final PhotonCamera photonCamera;
    private final SwerveSubsystem driveBase;

    private Double camYawToSpeaker = null;

    static Optional<Alliance> color;

    public static final Transform3d AprilTag_Robot_to_camera = new Transform3d(new Translation3d(inchesToMeters(11.0), 0, 0), new Rotation3d(0, degreesToRadians(-75), 0));
    public static final Transform2d Camera_To_Robot = new Transform2d(new Translation2d(inchesToMeters(11.0), 0), new Rotation2d(0.0));
    private final SwerveDrivePoseEstimator poseEstimator;

    private static final Matrix<N3, N1> SingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1 > MultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    // Set Target Speaker Positions
    public static Translation2d blueSpeakerPos = new Translation2d(0.076, 5.547868);
    public static Translation2d redSpeakerPos = new Translation2d(16.465042, 5.547868);

    private static final int Tag_To_Align = 7;

    private final Field2d field2d = new Field2d();

    private PhotonPoseEstimator photonPoseEstimator;


    public VisionSubsystem(PhotonCamera m_PhotonCamera, SwerveSubsystem m_SwerveSubsystem) {
        this.driveBase = m_SwerveSubsystem;
        this.photonCamera = m_PhotonCamera;

        photonPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, AprilTag_Robot_to_camera);

        /**try {
            photonPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, AprilTag_Robot_to_camera);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }*/

        poseEstimator = new SwerveDrivePoseEstimator(driveBase.getKinematics(), driveBase.getHeading(), driveBase.getSwerveModulePositions(), driveBase.getPose());
        //poseEstimator = driveBase.getSwerveDrivePoseEstimator();

        SmartDashboard.putData("VisionField", field2d);

        color = DriverStation.getAlliance();
    }

    @Override
    public void periodic() {
        var photonPoseEstimated = photonPoseEstimator.update();

        if (photonPoseEstimated.isPresent()) {
            var visionMeasurement = photonPoseEstimated.get();
            poseEstimator.addVisionMeasurement(visionMeasurement.estimatedPose.toPose2d(), visionMeasurement.timestampSeconds);
            //poseEstimator.addVisionMeasurement(visionMeasurement.estimatedPose.toPose2d(), visionMeasurement.timestampSeconds, getEstimatedStdDevs(visionMeasurement.estimatedPose.toPose2d(), photonCamera.getLatestResult().getTargets(), photonPoseEstimator.getFieldTags()));
            //driveBase.addCustomVisionMeasurement(visionMeasurement.estimatedPose.toPose2d(), visionMeasurement.timestampSeconds, getEstimatedStdDevs(visionMeasurement.estimatedPose.toPose2d(), photonCamera.getLatestResult().getTargets(), photonPoseEstimator.getFieldTags()));
            driveBase.addCustomVisionMeasurement(visionMeasurement.estimatedPose.toPose2d(), visionMeasurement.timestampSeconds);
        }
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), driveBase.getHeading(), driveBase.getSwerveModulePositions());

        field2d.setRobotPose(getCurrentPose());

        if (color.get() == Alliance.Blue) {
            camYawToSpeaker = normalizeAngle(getCurrentPose().getTranslation().minus(blueSpeakerPos).getAngle().getDegrees() + 0.0 );
        } else {
            camYawToSpeaker = normalizeAngle(getCurrentPose().getTranslation().minus(redSpeakerPos).getAngle().getDegrees() + 0.0);
        }

        SmartDashboard.putNumber("Cam to Speaker", camYawToSpeaker);
    }

    public Double getCamYawToSpeaker() {
        return camYawToSpeaker;
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public boolean hasTargets() {
        return photonPoseEstimator.update().isPresent();
    }

    public static Matrix<N3, N1> getEstimatedStdDevs(Pose2d estimatedPose, List<PhotonTrackedTarget> targets, AprilTagFieldLayout fieldLayout) {
        var estStdDevs = SingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;
        for(var tgt : targets) {
            var tagPose = fieldLayout.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
            continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) {
            return estStdDevs;
        }

        avgDist /= numTags;

        if (numTags > 1) {
            estStdDevs = MultiTagStdDevs;
        }

        if (numTags == 1 && avgDist > 5) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        return estStdDevs;
    }

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

    public int getTagToTarget() {
        return Tag_To_Align;
    }
}
