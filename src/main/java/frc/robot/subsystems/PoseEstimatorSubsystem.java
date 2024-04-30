package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PoseEstimatorSubsystem extends SubsystemBase{
    private final PhotonCamera photonCamera;
    private final SwerveSubsystem driveBase;

    public static final Transform3d AprilTag_Robot_to_camera = new Transform3d(new Translation3d(inchesToMeters(11.0), 0, inchesToMeters(8.0)), new Rotation3d(0, degreesToRadians(-75), 0));
    public static final Transform2d Camera_To_Robot = new Transform2d(new Translation2d(inchesToMeters(11.0), 0), new Rotation2d(0.0));

    private final SwerveDrivePoseEstimator poseEstimator;

    private static final Matrix<N3, N1> SingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1 > MultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    private PhotonPoseEstimator photonPoseEstimator;

    private double avgDist = 0;
    private int numTags = 0;

    private final Field2d field2d;

    private final AprilTagFieldLayout aprilTagField = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    /**StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose3d.struct).publish();

    StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();*/

    public PoseEstimatorSubsystem(PhotonCamera m_PhotonCamera, SwerveSubsystem m_SwerveSubsystem) {
        this.driveBase = m_SwerveSubsystem;
        this.photonCamera = m_PhotonCamera;

        field2d = driveBase.getSwerveField();

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagField, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, AprilTag_Robot_to_camera);

        poseEstimator = new SwerveDrivePoseEstimator(driveBase.getKinematics(), driveBase.getRawYaw(), driveBase.getSwerveModulePositions(), driveBase.getPose());

        /**try {
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagField, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, AprilTag_Robot_to_camera);
            //poseEstimator = driveBase.getSwerveDrivePoseEstimator();
        } catch(IOException e) {
            e.printStackTrace();
        }*/
        //poseEstimator = m_DrivePoseEstimator;
    }

    @Override
    public void periodic() {

        var EstTagStdDevs = SingleTagStdDevs;

        var photonPoseEstimated = photonPoseEstimator.update();

        if (photonPoseEstimated.isPresent()) {
            var visionMeasurement = photonPoseEstimated.get();

            var targets = photonCamera.getLatestResult().getTargets();

            for(var tgt : targets) {
                var tagPose = aprilTagField.getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                continue;

                numTags++;
                avgDist += tagPose.get().toPose2d().getTranslation().getDistance(visionMeasurement.estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                EstTagStdDevs = SingleTagStdDevs;
            } else {
                avgDist /= numTags;

                if (numTags > 1) {
                    EstTagStdDevs = MultiTagStdDevs;
                }

                if (numTags == 1 && avgDist > 5) {
                    EstTagStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                } else {
                    EstTagStdDevs = EstTagStdDevs.times(1 + (avgDist * avgDist / 30));
                }

                //poseEstimator.addVisionMeasurement(visionMeasurement.estimatedPose.toPose2d(), visionMeasurement.timestampSeconds, EstTagStdDevs);
            }

            //poseEstimator.setVisionMeasurementStdDevs(EstTagStdDevs);
            //poseEstimator.addVisionMeasurement(visionMeasurement.estimatedPose.toPose2d(), visionMeasurement.timestampSeconds);
            //driveBase.addCustomVisionReading(visionMeasurement.estimatedPose.toPose2d(), visionMeasurement.timestampSeconds);
            poseEstimator.addVisionMeasurement(visionMeasurement.estimatedPose.toPose2d(), visionMeasurement.timestampSeconds, EstTagStdDevs);
            driveBase.addCustomVisionReading(visionMeasurement.estimatedPose.toPose2d(), visionMeasurement.timestampSeconds, EstTagStdDevs);
        }

        /**if (RobotState.isAutonomous()) {
            poseEstimator.addVisionMeasurement(visionMeasurement.estimatedPose.toPose2d(), visionMeasurement.timestampSeconds);
        }*/

        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), driveBase.getHeading(), driveBase.getSwerveModulePositions());

        
        //Pose3d poseA = photonPoseEstimator.update().get().estimatedPose;
        //Pose3d poseA = new Pose3d(getCurrentPose()).transformBy(AprilTag_Robot_to_camera).transformBy(photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget());
        //publisher.set(poseA);

        field2d.setRobotPose(getCurrentPose());
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

}
