package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ControlChoice {

    private final RobotContainer robotContainer;
    
    public SendableChooser<Command> controlChooser = new SendableChooser<>();


    public ControlChoice(RobotContainer m_RobotContainer) {
        this.robotContainer = m_RobotContainer;

        controlChooser.setDefaultOption("Default", configureCameronBindings());
        controlChooser.addOption("Cameron", configureCameronBindings());
        controlChooser.addOption("Standard", configureStandardBindings());
        controlChooser.addOption("PS5", configurePS5Bindings());
        controlChooser.addOption("Only Joysticks", configureNoXboxBindings());
        controlChooser.addOption("Grace", configureGraceBindings());

    }

    public Command configureCameronBindings() {
        return new InstantCommand(() -> robotContainer.configure_Cameron_Bindings());
    }

    public Command configurePS5Bindings() {
        return new InstantCommand(() -> robotContainer.configure_PS5_Bindings());
    }

    public Command configureNoXboxBindings() {
        return new InstantCommand(() -> robotContainer.configure_NoXbox_Bindings());
    }

    public Command configureGraceBindings() {
        return new InstantCommand(() -> robotContainer.configure_Grace_Bindings());
    }

    public Command configureStandardBindings() {
        return new InstantCommand(() -> robotContainer.configureBindings());
    }

    public Command getControlChooserSelection() {
        return controlChooser.getSelected();
    }

}
