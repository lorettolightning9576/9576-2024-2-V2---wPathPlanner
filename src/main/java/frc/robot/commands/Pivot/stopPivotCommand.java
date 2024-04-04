package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class stopPivotCommand extends Command{
    private PivotSubsystem pivotSubsystem;

    public stopPivotCommand(PivotSubsystem m_PivotSubsystem) {
        this.pivotSubsystem = m_PivotSubsystem;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        pivotSubsystem.stopAimAndMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
