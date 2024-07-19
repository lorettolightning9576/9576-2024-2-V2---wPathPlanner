package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotTriggerCommandV2 extends Command{
    private PivotSubsystem pivotSubsystem;

    public double axisValue;

    public PivotTriggerCommandV2(PivotSubsystem m_PivotSubsystem, double m_axisValue) {
        this.pivotSubsystem = m_PivotSubsystem;
        addRequirements(pivotSubsystem);
        this.axisValue = m_axisValue;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        pivotSubsystem.setArmTriggerSpeed(axisValue);
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.stopAimAndMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
