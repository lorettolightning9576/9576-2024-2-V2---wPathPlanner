package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Photon;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_NAMES;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

public class TelopDriveWithAimCommand extends Command{
  private final SwerveSubsystem  swerve;
  private final DoubleSupplier   vX;
  private final DoubleSupplier   vY;
  private final DoubleSupplier   omega;
  private final BooleanSupplier  driveMode;
  private final SwerveController controller;
  private final VisionSubsystem visionSubsystem;

  /**private final Thread photonThread = new Thread(new Photon(APRILTAG_CAMERA_NAMES, ROBOT_TO_CAMERA_TRANSFORMS, 
  swerve::addVisionMeasurement, () -> swerve.getPose()));*/

  public TelopDriveWithAimCommand(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, BooleanSupplier driveMode, VisionSubsystem m_vision) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;
    this.controller = swerve.getSwerveController();
    this.visionSubsystem = m_vision;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double xVelocity = Math.pow(vX.getAsDouble(), 3);
    double yVelocity = Math.pow(vY.getAsDouble(), 3);
    double angVelocity = Math.pow(omega.getAsDouble(), 3);

    /**var color = DriverStation.getAlliance();
    if (color.isPresent()) {
        if (color.get() == Alliance.Red) {
            xVelocity = -xVelocity;
            yVelocity = -yVelocity;
        }
    }*/

    SmartDashboard.putNumber("vX",xVelocity);
    SmartDashboard.putNumber("vY", yVelocity);
    SmartDashboard.putNumber("omega", angVelocity);

    // TODO: Update SwerveSubsystem to include boolean "AreWeAiming"
    // TODO: Update new teleopDriveWithAim command to check "areweaiming" and "do I
    // see the right target?"

    if (swerve.getAreWeAiming()) {
      Double headingToTag = visionSubsystem.getCamYawToSpeaker();

      if (headingToTag == null) {
        SmartDashboard.putString("aimDrive.doIHaveTarget", "No");
        angVelocity = 0;
      } else {
        // TODO: Currently, this points the FRONT of the robot toward the apriltag.
        // We'll need to modify this to point the rear of the robot to the AprilTag
        // when we get JoeHann back.

        double currentPosRotation = swerve.getPose().getRotation().getDegrees();
        double targetHeading = visionSubsystem.normalizeAngle(headingToTag + 180);

        angVelocity = controller.headingCalculate(degreesToRadians(currentPosRotation), 
                                                  degreesToRadians(targetHeading));
        angVelocity = MathUtil.clamp(angVelocity, -swerve.getMaximumAngularVelocity(), swerve.getMaximumAngularVelocity());

        //Prints on dashboard
        SmartDashboard.putString("aimDrive.doIHaveTarget", "Yes");

        SmartDashboard.putNumber("aimDrive.currentRobotRotation", currentPosRotation);
        SmartDashboard.putNumber("aimDrive.headingToTag", headingToTag);
        SmartDashboard.putNumber("aimDrive.targetHeading", targetHeading);
        SmartDashboard.putNumber("aimDrive.lastAngVelocity", angVelocity);

      }
    } else {
      angVelocity = angVelocity * swerve.getMaximumAngularVelocity();
    }
    SmartDashboard.putNumber("aimDrive.angVelocity", angVelocity);

    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed), 
          angVelocity,
          driveMode.getAsBoolean()
    );
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0.0 * swerve.maximumSpeed, 0.0 * swerve.maximumSpeed), 
      0.0 * controller.config.maxAngularVelocity,
      driveMode.getAsBoolean());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
