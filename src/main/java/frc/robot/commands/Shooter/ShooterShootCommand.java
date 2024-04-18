// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterShootCommand extends Command {
  ShooterSubsystem shooterSubsystem;
  PivotSubsystem pivotSubsystem;
  
  public ShooterShootCommand(ShooterSubsystem m_ShooterSubsystem, PivotSubsystem m_PivotSubsystem) {
    this.shooterSubsystem = m_ShooterSubsystem;
    this.pivotSubsystem = m_PivotSubsystem;
    addRequirements(shooterSubsystem, pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setShooterShoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pivotSubsystem.isTooLow()) {
      return true;
    } else {
      return false;
    }
  }
}
