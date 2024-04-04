package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeOutCommand extends Command{
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    
    public IntakeOutCommand(IntakeSubsystem m_IntakeSubsystem, ShooterSubsystem m_ShooterSubsystem) {
        this.intakeSubsystem = m_IntakeSubsystem;
        this.shooterSubsystem = m_ShooterSubsystem;
        addRequirements(intakeSubsystem);
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeReverse();
        shooterSubsystem.setShooterCustomSpeed(-0.2);

    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
        shooterSubsystem.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
