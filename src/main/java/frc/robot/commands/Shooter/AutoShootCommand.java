// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends Command {
  Timer timer = new Timer();
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;

  public AutoShootCommand(IntakeSubsystem m_IntakeSubsystem, ShooterSubsystem m_ShooterSubsystem) {

    this.intakeSubsystem = m_IntakeSubsystem;
    this.shooterSubsystem = m_ShooterSubsystem;

    addRequirements(intakeSubsystem);
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setShooterShoot();
    
    if (timer.hasElapsed(1)) {
      intakeSubsystem.setIntakeFeed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
    shooterSubsystem.stopShooter();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(Constants.GeneralConstants.autoShootEndTime)) {
      return true;
    } else {
      return false;
    }
  }
}

