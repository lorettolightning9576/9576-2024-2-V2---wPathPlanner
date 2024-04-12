package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.FIELD_LENGTH;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH;
import static frc.robot.Constants.VisionConstants.SINGLE_TAG_DISTANCE_THRESHOLD;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.Constants.VisionConstants;

public class Photon implements Runnable{
    public final PhotonPoseEstimator[] photonPoseEstimators;

    public final RawSubscriber[] rawBytesSubscribers;

    // These Standard Deviations can be increased to "trust" vision measurements more. They are scaled based distance.
    /** Single tag standard deviation at 1-meter */
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
    /** Multitag standard deviation at 1-meter */
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

    private final int[] waitHandles;

    private final Packet packet = new Packet(1);

    public final Supplier<Pose2d> poseSupplier;

    @SuppressWarnings("unchecked")
    private final StructArrayPublisher<AprilTag>[] aprilTagPublishers = new StructArrayPublisher[2];

    private final AddVisionMeasurement poseConsumer;

    public Photon(String[] cameraNames, Transform3d[] m_robotToCameras, AddVisionMeasurement m_poseConsumer, Supplier<Pose2d> m_poseSupplier) {
        this.poseConsumer = m_poseConsumer;
        this.poseSupplier = m_poseSupplier;

        rawBytesSubscribers = new RawSubscriber[cameraNames.length];
        photonPoseEstimators = new PhotonPoseEstimator[cameraNames.length];
        waitHandles = new int[cameraNames.length];

        var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        for (int i = 0; i < cameraNames.length; i++) {
            var cameraTable = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(cameraNames[i]);
            rawBytesSubscribers[i] = cameraTable.getRawTopic("rawBytes")
                .subscribe(
                    "rawBytes", new byte[] {}, PubSubOption.periodic(0.01),PubSubOption.sendAll(true)
                );
            waitHandles[i] = rawBytesSubscribers[i].getHandle();
            photonPoseEstimators[i] = new PhotonPoseEstimator(
                layout, MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera(cameraNames[i]), m_robotToCameras[i]);
        }
    }

    @Override
    public void run() {
        var emptyAprilTagArray = new AprilTag[0];
        while (!Thread.interrupted()) {
            int[] signaledHandles = null;
            try {
                signaledHandles = WPIUtilJNI.waitForObjects(waitHandles);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            if (RobotState.isAutonomous()) {
                continue;
            }

            for (int i = 0; i < signaledHandles.length; i++) {
                int cameraIndex = getCameraIndex(signaledHandles[i]);
                var aprilTagPublisher = aprilTagPublishers[cameraIndex];
                var photonPoseEstimator = photonPoseEstimators[cameraIndex];

                var photonResults = getLatestResult(cameraIndex);
                if (photonResults.hasTargets() && (photonResults.targets.size() > 1 || (photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD))) {
                    photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
                        var estimatedPose = estimatedRobotPose.estimatedPose;
                        //Make sure the measurement is on the field
                        if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH.in(Meters)
                            && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH.in(Meters)) {
                                var stdDevs = getEstimationStdDevs(
                                    estimatedPose.toPose2d(), photonResults.getTargets(), photonPoseEstimator.getFieldTags());
                            poseConsumer.addVisionMeasurement(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds, stdDevs);
                        }
                    });
                } else {
                    //No tags, send empty array to NT
                    aprilTagPublisher.accept(emptyAprilTagArray);
                }
            }
        }
        Arrays.stream(rawBytesSubscribers).forEach(RawSubscriber::close);
        Arrays.stream(aprilTagPublishers).forEach(StructArrayPublisher::close);
    }

    /**
    * Transform a target from PhotonVision to a pose on the field
    * @param target target data from PhotonVision
    * @param robotPose current pose of the robot
    * @param robotToCamera transform from robot to the camera that saw the target
    * @return an AprilTag with an ID and pose
    */
    /**private static AprilTag getTargetPose(PhotonTrackedTarget target, Pose2d robotPose, Transform3d robotToCamera) {
        var targetPose = new Pose3d(robotPose)
            .transformBy(robotToCamera)
            .transformBy(target.getBestCameraToTarget());
        return new AprilTag(target.getFiducialId(), targetPose);
    } */

    /**
    * Find the camera index for a table wait handle
    * @param signaledHandle handle
    * @return index, or -1 if not found
    */
    public int getCameraIndex(int signaledHandle) {
        for (int i = 0; i < waitHandles.length; i++) {
            if (waitHandles[i] == signaledHandle) {
                return i;
            }
        }
        return -1;
    }

    /**
    * Scales the standard deviation based on the number of targets and their distance.
    *
    * @param estimatedPose estimated pose
    * @param targets targets from PhotonVision
    * @param fieldLayout tag poses
    */
    public static Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, List<PhotonTrackedTarget> targets, AprilTagFieldLayout fieldLayout) {

        var estStdDevs = SINGLE_TAG_STD_DEVS;
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
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

        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            estStdDevs = MULTI_TAG_STD_DEVS;
        }

        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > SINGLE_TAG_DISTANCE_THRESHOLD.in(Meters)) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        return estStdDevs;
    }

    public PhotonPipelineResult getLatestResult(int cameraIndex) {
        packet.clear();
        var result = new PhotonPipelineResult();
        packet.setData(rawBytesSubscribers[cameraIndex].get(new byte[] {}));

        if (packet.getSize() < 1) {
            return result;
        }

        result = PhotonPipelineResult.serde.unpack(packet);
        result.setTimestampSeconds((rawBytesSubscribers[cameraIndex].getLastChange() / 1e6) - result.getLatencyMillis() / 1e3);
        return result;
    }

    @FunctionalInterface
    public interface AddVisionMeasurement {
    void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
