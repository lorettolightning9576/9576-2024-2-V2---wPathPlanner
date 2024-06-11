package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Colors;

public class Blinkin extends SubsystemBase {
    public static final Spark leftBlinkin = new Spark(3);
    public static final Spark rightBlinkin = new Spark(5);
    public static final Colors colors = new Colors();

    private Timer timer = new Timer();
    private boolean refresh = false;

    private double desiredColor = 0.0;

    public void addDashboardWidgets(ShuffleboardLayout layout) {
        layout.withProperties(Map.of("Number of columns", 3, "Number of rows", 2));
    }

    public void setBlackCommand() {
        desiredColor = colors.black;
    }

    public void setCustomColor(double color) {
        desiredColor = color;
    }

    @Override
    public void periodic() {
        if (desiredColor != 0) {
            leftBlinkin.set(desiredColor);
            rightBlinkin.set(desiredColor);
        }
    }
}
