package frc.robot.commands.swervedrive.drivebase;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import org.photonvision.PhotonCamera;

import swervelib.SwerveController;

public class TelopDrive_wVisionAlign extends Command{
  private final SwerveSubsystem  swerve;
  private final DoubleSupplier   vX;
  private final DoubleSupplier   vY;
  private final DoubleSupplier   omega;
  private final BooleanSupplier  driveMode;
  private final SwerveController controller;
  private final VisionSubsystem vision;
  private final PhotonCamera photonCamera;
  static Optional<Alliance> color;
  private Double camYawToSpeaker = null;

  public TelopDrive_wVisionAlign(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, BooleanSupplier driveMode, VisionSubsystem m_vision, PhotonCamera m_PhotonCamera) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;
    this.controller = swerve.getSwerveController();
    this.vision = m_vision;
    this.photonCamera = m_PhotonCamera;
    addRequirements(swerve);

    color = DriverStation.getAlliance();
  }

  @Override
  public void execute() {
    double xVelocity = Math.pow(vX.getAsDouble(), 3);
    double yVelocity = Math.pow(vY.getAsDouble(), 3);
    double angVelocity = Math.pow(omega.getAsDouble(), 3);
    SmartDashboard.putNumber("vX",xVelocity);
    SmartDashboard.putNumber("vY", yVelocity);
    SmartDashboard.putNumber("omega", angVelocity);

    if (swerve.getAreWeAiming()) {

      var photonResults = photonCamera.getLatestResult();
      var targetOpt = photonResults.getTargets().stream().filter(t -> t.getFiducialId() == 4).findFirst();
      var target = targetOpt.get();

      var camToTarget = target.getBestCameraToTarget();
      var transform = new Transform2d(camToTarget.getTranslation().toTranslation2d(), camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));
      var cameraPose = swerve.getPose().transformBy(transform);

      if (color.get() == Alliance.Blue) {
        camYawToSpeaker = VisionSubsystem.normalizeAngle(cameraPose.getTranslation().minus(VisionSubsystem.blueSpeakerPos).getAngle().getDegrees());
      } else {
        camYawToSpeaker = VisionSubsystem.normalizeAngle(cameraPose.getTranslation().minus(VisionSubsystem.redSpeakerPos).getAngle().getDegrees());
      }

      if (camYawToSpeaker == null) {
        angVelocity = 0;
      } else {
        double currentPosRotation = swerve.getPose().getRotation().getDegrees();
        double targetHeading = VisionSubsystem.normalizeAngle(camYawToSpeaker);
        angVelocity = controller.headingCalculate(degreesToRadians(currentPosRotation), degreesToRadians(targetHeading));

        angVelocity = MathUtil.clamp(angVelocity, -swerve.getMaximumAngularVelocity(), swerve.getMaximumAngularVelocity());
      }
    } else {
      angVelocity = angVelocity*swerve.getMaximumAngularVelocity();
    }

    

    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed), 
          angVelocity,
          driveMode.getAsBoolean());
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
