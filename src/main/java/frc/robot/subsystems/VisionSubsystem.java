package frc.robot.subsystems;

import java.io.IOException;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase{
    private final PhotonCamera photonCamera;
    private final SwerveSubsystem driveBase;

    public static final Transform3d AprilTag_Robot_to_camera = new Transform3d(new Translation3d(inchesToMeters(11.0), 0, 0), new Rotation3d(0, degreesToRadians(-75), 0));
    public static final Transform2d Camera_To_Robot = new Transform2d(new Translation2d(inchesToMeters(11.0), 0), new Rotation2d(0.0));
    private final SwerveDrivePoseEstimator poseEstimator;

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

        SmartDashboard.putData("VisionField", field2d);
    }

    @Override
    public void periodic() {
        var photonPoseEstimated = photonPoseEstimator.update();

        if (photonPoseEstimated.isPresent()) {
            var visionMeasurement = photonPoseEstimated.get();
            poseEstimator.addVisionMeasurement(visionMeasurement.estimatedPose.toPose2d(), visionMeasurement.timestampSeconds);
            driveBase.addCustomVisionMeasurement(visionMeasurement.estimatedPose.toPose2d(), visionMeasurement.timestampSeconds);
        }
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), driveBase.getHeading(), driveBase.getSwerveModulePositions());

        field2d.setRobotPose(getCurrentPose());
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }
}
