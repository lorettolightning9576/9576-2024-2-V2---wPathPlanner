package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoTest extends SubsystemBase{
    
    public static Servo servofive;

    public ServoTest() {
        servofive = new Servo(2);
        //servofive.setBoundsMicroseconds(2500, 0, 1500, 0, 500);
        //servofive.setPeriodMultiplier(PeriodMultiplier.k4X);
    }

    public void addDashboardWidgets(ShuffleboardLayout layout) {
        layout.withProperties(Map.of("Number of columns", 2, "Number of rows", 4));
        layout.addNumber("Servo Angle", servofive::getAngle)                    .withPosition(0, 0);
        layout.addNumber("Servo Channel", servofive::getChannel)                .withPosition(0, 1);
        layout.addNumber("Servo Position", servofive::getPosition)              .withPosition(0, 2);
        layout.addNumber("Servo Speed", servofive::getSpeed)                    .withPosition(0, 3);
        layout.addNumber("Servo get", servofive::get)                           .withPosition(1, 0);
        layout.addNumber("Servo Pulse", servofive::getPulseTimeMicroseconds)    .withPosition(1, 1);
        layout.addNumber("Servo Handle", servofive::getHandle)                  .withPosition(1, 2);
    }

    public void setServoPosition(double position) {
        servofive.setPosition(position);
    }

    public void setServoAngle(double angle) {
        servofive.setAngle(angle);
    }

    public void setServoSet(double value) {
        servofive.set(value);
    }

    public void setServoSpeed(double speed) {
        servofive.setSpeed(speed);
    }

    public void setServoDisabled() {
        servofive.setDisabled();
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

    public Command setServoDisableCommand() {
        return run(() -> setServoDisabled());
    }

}
