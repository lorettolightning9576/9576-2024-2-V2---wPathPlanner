package frc.robot.commands.swervedrive.drivebase;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;

public class AimAtTag extends Command{

    private final PhotonCamera photonCamera;
    private final SwerveSubsystem swerve;

    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new Constraints(2.0, 2.0);

    private double Angular_P = 0.5;
    private double Angular_D = 0.00;
    private ProfiledPIDController turnController = new ProfiledPIDController(Angular_P, 0.0, Angular_D, OMEGA_CONSTRAINTS);

    private Pose2d goalPose;
    private PhotonTrackedTarget lastTarget;

    private double rotationSpeed = 0.0;

    private final Supplier<Pose2d> poseProvider;

    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier omega;
    private final BooleanSupplier driveMode;
    private final SwerveController controller;

    public AimAtTag(PhotonCamera m_PhotonCamera, SwerveSubsystem m_SwerveSubsystem, DoubleSupplier m_vX, DoubleSupplier m_vY, DoubleSupplier m_omega, BooleanSupplier m_driveMode, Supplier<Pose2d> pose2d) {
        this.photonCamera = m_PhotonCamera;
        this.swerve = m_SwerveSubsystem;
        this.vX = m_vX;
        this.vY = m_vY;
        this.omega = m_omega;
        this.driveMode = m_driveMode;
        this.controller = swerve.getSwerveController();
        this.poseProvider = pose2d;
        addRequirements(swerve);

        turnController.setTolerance(degreesToRadians(5));
        turnController.enableContinuousInput(-1, 1);
    }

    @Override
    public void initialize() {
        goalPose = null;
        lastTarget = null;

        var robotpose = poseProvider.get();
        turnController.reset(robotpose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        double xVelocity = Math.pow(vX.getAsDouble(), 3);
        double yVelocity = Math.pow(vY.getAsDouble(), 3);
        double angVelocity = Math.pow(omega.getAsDouble(), 3);

        var robotPose = poseProvider.get();
        var results = photonCamera.getLatestResult();

        if (results.hasTargets()) {
            var targetTag = results.getTargets().stream().filter(t -> t.getFiducialId() == 7).findFirst();

            if (targetTag.isPresent()) {
                var target = targetTag.get();
                if (!target.equals(lastTarget)) {
                    lastTarget = target;

                    var camToTarget = target.getBestCameraToTarget();
                    var transform = new Transform2d(camToTarget.getTranslation().toTranslation2d(), camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));

                    var cameraPose = robotPose.transformBy(PoseEstimatorSubsystem.Camera_To_Robot.inverse());
                    Pose2d targetPose = cameraPose.transformBy(transform);

                    goalPose = targetPose.rotateBy(Rotation2d.fromDegrees(180));
                }

                if (null != goalPose) {
                    turnController.setGoal(goalPose.getRotation().getRadians());
                }

            } else {
                rotationSpeed = angVelocity * controller.config.maxAngularVelocity;
            }

            rotationSpeed = turnController.calculate(robotPose.getRotation().getRadians());

            rotationSpeed = rotationSpeed * controller.config.maxAngularVelocity;

        } else {
            rotationSpeed = angVelocity * controller.config.maxAngularVelocity;
        }

        xVelocity = xVelocity * swerve.maximumSpeed;
        yVelocity = yVelocity * swerve.maximumSpeed;

        /**swerve.drive(new Translation2d(xVelocity, yVelocity), 
            rotationSpeed,
            driveMode.getAsBoolean()
        );*/

        if (turnController.atGoal()) {
            rotationSpeed = angVelocity * controller.config.maxAngularVelocity;
        } else {
            rotationSpeed = turnController.calculate(robotPose.getRotation().getRadians());
            rotationSpeed = rotationSpeed * controller.config.maxAngularVelocity;
        }

        swerve.setVisionChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationSpeed, robotPose.getRotation()));

        //swerve.drive(new Translation2d(xVelocity * swerve));
    }

    @Override
    public void end(boolean interrupted) {
      swerve.drive(new Translation2d(0 * swerve.maximumSpeed, 0 * swerve.maximumSpeed), 
        0 * controller.config.maxAngularVelocity,
        driveMode.getAsBoolean());
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
