package frc.robot.commands.swervedrive.drivebase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PhotonAlignCommand extends Command{
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new Constraints(0.75, 1);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new Constraints(0.75, 1);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new Constraints(0.75, 1);

    public static final int Tag_To_Align = 7;
    private static final Transform2d Tag_To_Goal = new Transform2d(new Translation2d(1.0, 0.0), Rotation2d.fromDegrees(180));
    private static final Rotation2d Tag_To_Goal_Rotation = new Rotation2d(degreesToRadians(180));

    private final PhotonCamera photonCamera;
    private final SwerveSubsystem driveBase;
    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(2, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(4, 0, 0, OMEGA_CONSTRAINTS);

    private Pose2d goalPose;
    private PhotonTrackedTarget lastTarget;

    public PhotonAlignCommand(PhotonCamera m_PhotonCamera, SwerveSubsystem m_SwerveSubsystem, Supplier<Pose2d> pose2d) {
        addRequirements(m_SwerveSubsystem);
        driveBase = m_SwerveSubsystem;
        this.photonCamera = m_PhotonCamera;
        this.poseProvider = pose2d;

        xController.setTolerance(0.1);
        yController.setTolerance(0.1);
        omegaController.setTolerance(degreesToRadians(3));
        omegaController.enableContinuousInput(-1, 1);
    }

    @Override
    public void initialize() {
        goalPose = null;
        lastTarget = null;
        var robotPose = driveBase.getPose();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        var robotPose = driveBase.getPose();
        var photonResults = photonCamera.getLatestResult();
        if (photonResults.hasTargets()) {
            var targetOpt = photonResults.getTargets().stream().filter(t -> t.getFiducialId() == 7).findFirst();
            if (targetOpt.isPresent()) {
                var target = targetOpt.get();
                if (!target.equals(lastTarget)) {
                    lastTarget = target;
                    var camToTarget = target.getBestCameraToTarget();
                    var transform = new Transform2d(camToTarget.getTranslation().toTranslation2d(), camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));
                    var cameraPose = robotPose.transformBy(VisionSubsystem.Camera_To_Robot.inverse());
                    Pose2d targetPose = cameraPose.transformBy(transform);

                    goalPose = targetPose.transformBy(Tag_To_Goal);
                    //goalPose = targetPose.rotateBy(Tag_To_Goal_Rotation);
                }

                if (null != goalPose) {
                    xController.setGoal(goalPose.getX());
                    yController.setGoal(goalPose.getY());
                }
            }
        } else {
        driveBase.drive(new Translation2d(0, 0), 0, false);
        }

        var xSpeed = xController.calculate(robotPose.getX());
        if (xController.atGoal()) {
            xSpeed = 0;
        }

        var ySpeed = yController.calculate(robotPose.getY());
        if (yController.atGoal()) {
            ySpeed = 0;
        }

        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }

        driveBase.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, omegaSpeed, robotPose.getRotation()));
        //driveBase.driveFieldOriented(new ChassisSpeeds(0, 0, omegaSpeed));

    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(new Translation2d(0, 0), 0, false);
    }
}
