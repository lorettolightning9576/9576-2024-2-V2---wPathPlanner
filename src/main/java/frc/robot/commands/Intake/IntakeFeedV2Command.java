// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeFeedV2Command extends Command {
  IntakeSubsystem intakeSubsystem;

  public IntakeFeedV2Command(IntakeSubsystem m_IntakeSubsystem) {
    this.intakeSubsystem = m_IntakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setIntakeFeed();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
  }

  @Override
  public boolean isFinished() {
    /**if (xboxControllerCommand.rightTrigger().getAsBoolean() && xboxControllerCommand.leftTrigger().getAsBoolean()) {
      return false;
    } else if (xboxControllerCommand.leftBumper().getAsBoolean()) {
      return false;
    } else if (intakeSubsystem.hasNoteRAW()) {
      return true;
    } else {
      return false;
    }*/

    return false;
  }
}