package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.dataflow.structures.Packet;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.StructArrayPublisher;

public class Photon implements Runnable{
    private final PhotonPoseEstimator[] photonPoseEstimators;

    private final RawSubscriber[] rawBytesSubscribers;

    private final int[] waitHandles;

    private final Packet packet = new Packet(1);

    private final Supplier<Pose2d> poseSupplier;

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

    @FunctionalInterface
    public interface AddVisionMeasurement {
    void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
