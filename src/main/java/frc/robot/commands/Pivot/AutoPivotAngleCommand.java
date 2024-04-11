// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoPivotAngleCommand extends Command {
  private PivotSubsystem pivotSubsystem;
  private SwerveSubsystem swerveSubsystem;
  /** Creates a new AutoPivotAngleCommand. */
  public AutoPivotAngleCommand(PivotSubsystem m_PivotSubsystem) {
    this.pivotSubsystem = m_PivotSubsystem;
    addRequirements(pivotSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Constants.shooterAngleMap.get(swerveSubsystem.getDistanceToSpeaker());
    SmartDashboard.putNumber("Calculated Shooter Angle", Constants.shooterAngleMap.get(swerveSubsystem.getDistanceToSpeaker()) );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
