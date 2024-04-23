// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.BooleanSupplier;

public class ShooterShootV2Command extends Command {
  ShooterSubsystem shooterSubsystem;
  BooleanSupplier pivotIsTooLow;
  
  public ShooterShootV2Command(ShooterSubsystem m_ShooterSubsystem, BooleanSupplier m_pivotIsTooLow) {
    this.shooterSubsystem = m_ShooterSubsystem;
    this.pivotIsTooLow = m_pivotIsTooLow;
    addRequirements(shooterSubsystem);
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
    if (RobotState.isTeleop() && pivotIsTooLow.getAsBoolean()) {
      return true;
    } else {
      return false;
    }
    //return false;
  }
}
