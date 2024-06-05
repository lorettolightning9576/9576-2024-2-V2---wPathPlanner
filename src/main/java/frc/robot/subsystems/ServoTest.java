package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoTest extends SubsystemBase{
    
    public Servo servo;

    public ServoTest() {
        servo = new Servo(5);
    }

    public void addDashboardWidgets(ShuffleboardLayout layout) {
        layout.withProperties(Map.of("Number of columns", 2, "Number of rows", 4));
        layout.addNumber("Servo Angle", servo::getAngle)                    .withPosition(0, 0);
        layout.addNumber("Servo Channel", servo::getChannel)                .withPosition(0, 1);
        layout.addNumber("Servo Position", servo::getPosition)              .withPosition(0, 2);
        layout.addNumber("Servo Speed", servo::getSpeed)                    .withPosition(0, 3);
        layout.addNumber("Servo get", servo::get)                           .withPosition(1, 0);
        layout.addNumber("Servo Pulse", servo::getPulseTimeMicroseconds)    .withPosition(1, 1);
        layout.addNumber("Servo Handle", servo::getHandle)                  .withPosition(1, 2);
    }

    public void setServoPosition(double position) {
        servo.setPosition(position);
    }

    public void setServoAngle(double angle) {
        servo.setAngle(angle);
    }

    public void setServoSet(double value) {
        servo.set(value);
    }

    public void setServoSpeed(double speed) {
        servo.setSpeed(speed);
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

}
