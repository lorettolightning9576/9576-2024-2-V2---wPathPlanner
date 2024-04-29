package frc.robot.commands.swervedrive.drivebase;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;

public class AimAtTag extends Command{

    private final PhotonCamera photonCamera;
    private final SwerveSubsystem swerve;

    private double Linear_P = 0.1;
    private double Linear_D = 0.0;

    private double Angular_P = 0.5;
    private double Angular_D = 0.00;

    private PIDController forwaPidController = new PIDController(Linear_D, 0.0, Linear_D);
    private ProfiledPIDController turnController = new ProfiledPIDController(Angular_P, 0.0, Angular_D, new TrapezoidProfile.Constraints(2.5, 2.5));



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

            var camToTarget = results.getBestTarget().getBestCameraToTarget();
            var transform = new Transform2d(camToTarget.getTranslation().toTranslation2d(), camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(170)));
            //var rotation = camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90));
            //var rotation = camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(170));

            var cameraPose = robotPose.transformBy(PoseEstimatorSubsystem.Camera_To_Robot.inverse());
            //Pose2d targetPose = cameraPose.rotateBy(rotation);\
            Pose2d targetPose = cameraPose.transformBy(transform);

            if (targetPose != null) {
                turnController.setGoal(targetPose.getRotation().getRadians());
            }

            rotationSpeed = turnController.calculate(robotPose.getRotation().getRadians());

            rotationSpeed = rotationSpeed * controller.config.maxAngularVelocity;

            //rotationSpeed = -turnController.calculate(results.getBestTarget().getYaw(), 180);
            //SmartDashboard.putNumber("Vision Speed", rotationSpeed);
            //rotationSpeed = rotationSpeed * controller.config.maxAngularVelocity;
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

        swerve.setVisionChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationSpeed, poseProvider.get().getRotation()));

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
