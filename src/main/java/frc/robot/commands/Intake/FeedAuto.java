// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FeedAuto extends Command {

  IntakeSubsystem intakeSubsystem;
  

  public FeedAuto(IntakeSubsystem m_IntakeSubsystem) {
    this.intakeSubsystem = m_IntakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSubsystem.setIntakeFeed();;
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
