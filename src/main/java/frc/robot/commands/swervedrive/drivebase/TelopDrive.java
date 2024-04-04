package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

public class TelopDrive extends Command{
  private final SwerveSubsystem  swerve;
  private final DoubleSupplier   vX;
  private final DoubleSupplier   vY;
  private final DoubleSupplier   omega;
  private final BooleanSupplier  driveMode;
  private final SwerveController controller;

  public TelopDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega, BooleanSupplier driveMode) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.driveMode = driveMode;
    this.controller = swerve.getSwerveController();
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

    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed), 
          angVelocity * controller.config.maxAngularVelocity,
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
