// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter.ShooterFun;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterFunV3Command extends Command {
  ShooterSubsystem shooterSubsystem;
  
  public ShooterFunV3Command(ShooterSubsystem m_ShooterSubsystem) {
    this.shooterSubsystem = m_ShooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    shooterSubsystem.setShooterFUN_Short();
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
