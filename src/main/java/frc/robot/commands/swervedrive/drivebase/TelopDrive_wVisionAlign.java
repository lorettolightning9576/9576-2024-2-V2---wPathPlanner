package frc.robot.commands.swervedrive.drivebase;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

public class TelopDrive_wVisionAlign extends Command{
  private final SwerveSubsystem  swerve;
  private final DoubleSupplier   vX;
  private final DoubleSupplier   vY;
  private final DoubleSupplier   omega;
  private final BooleanSupplier  driveMode;
  private final SwerveController controller;
  private final VisionSubsystem vision;

  public TelopDrive_wVisionAlign(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, BooleanSupplier driveMode, VisionSubsystem m_vision) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;
    this.controller = swerve.getSwerveController();
    this.vision = m_vision;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    double xVelocity = Math.pow(vX.getAsDouble(), 3);
    double yVelocity = Math.pow(vY.getAsDouble(), 3);
    double angVelocity = Math.pow(omega.getAsDouble(), 3);
    SmartDashboard.putNumber("vX",xVelocity);
    SmartDashboard.putNumber("vY", yVelocity);
    SmartDashboard.putNumber("omega", angVelocity);

    if (SwerveSubsystem.getAreWeAiming()) {

      Double headingToTag = vision.getCamYawToSpeaker();

      if (headingToTag == null) {
        angVelocity = 0;
      } else {
        double currentPosRotation = swerve.getPose().getRotation().getDegrees();
        double targetHeading = VisionSubsystem.normalizeAngle(headingToTag);
        angVelocity = controller.headingCalculate(degreesToRadians(currentPosRotation), 
                                                  degreesToRadians(targetHeading)
        );

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
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
