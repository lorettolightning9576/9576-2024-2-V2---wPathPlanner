package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeployServo extends SubsystemBase{
    
    public static Servo deployServo;

    public DeployServo() {
        deployServo = new Servo(5);
        //servofive.setBoundsMicroseconds(2500, 0, 1500, 0, 500);
        //servofive.setPeriodMultiplier(PeriodMultiplier.k4X);
    }

    public void addDashboardWidgets(ShuffleboardLayout layout) {
        layout.withProperties(Map.of("Number of columns", 2, "Number of rows", 4));
        layout.addNumber("Servo Angle", deployServo::getAngle)                    .withPosition(0, 0);
        layout.addNumber("Servo Channel", deployServo::getChannel)                .withPosition(0, 1);
        layout.addNumber("Servo Position", deployServo::getPosition)              .withPosition(0, 2);
        layout.addNumber("Servo Speed", deployServo::getSpeed)                    .withPosition(0, 3);
        layout.addNumber("Servo get", deployServo::get)                           .withPosition(1, 0);
        layout.addNumber("Servo Pulse", deployServo::getPulseTimeMicroseconds)    .withPosition(1, 1);
        layout.addNumber("Servo Handle", deployServo::getHandle)                  .withPosition(1, 2);
    }

    public void setServoPosition(double position) {
        deployServo.setPosition(position);
    }

    public void setServoAngle(double angle) {
        deployServo.setAngle(angle);
    }

    public void setServoSet(double value) {
        deployServo.set(value);
    }

    public void setServoSpeed(double speed) {
        deployServo.setSpeed(speed);
    }

    public void setServoDisabled() {
        deployServo.setDisabled();
    }

    public Command setServoSetCommand(double value) {
        return run(() -> setServoSet(value));
    }

    public Command setServoSpeedCommand(double speed) {
        return run(() -> setServoSpeed(speed));
    }

    public Command setServoAngleCommand(double angle) {
        return run(() -> setServoAngle(angle));
    }

    public Command setServoPositionCommand(double position) {
        return run(() -> setServoPosition(position));
    }

    public Command deployDownPositionCommand() {
        return this.runEnd(        
        () -> {
            setServoPosition(1.0);
        },
        () -> {
            setServoDisabled();
        }
        );
    }

    public Command deployUpPositionCommand() {
        return this.runEnd(        
        () -> {
            setServoPosition(0.0);
        },
        () -> {
            setServoDisabled();
        }
        );
    }

    public Command setServoDisableCommand() {
        return this.run(() -> setServoDisabled());
    }

}
