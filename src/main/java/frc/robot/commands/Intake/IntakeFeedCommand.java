// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeFeedCommand extends Command {

  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  

  public IntakeFeedCommand(IntakeSubsystem m_IntakeSubsystem, ShooterSubsystem m_shooterSubsystem) {
    this.intakeSubsystem = m_IntakeSubsystem;
    this.shooterSubsystem = m_shooterSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooterSubsystem.setShooterShoot();
    intakeSubsystem.setIntakeFeed();;
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
    intakeSubsystem.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}