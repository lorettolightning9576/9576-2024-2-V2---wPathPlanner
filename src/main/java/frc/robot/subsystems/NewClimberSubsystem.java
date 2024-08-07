// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NewClimberSubsystem extends SubsystemBase{
    
    private double raiseLimit = 210.0;

    private double RAISE_SPEED = 0.5;
    private double LOWER_SPEED = -0.5;

    private double fast_RAISE_SPEED = 0.75;
    private double fast_LOWER_SPEED = -0.75;

    public boolean fastLower = false;
    public boolean fastRaise = false;

    public static CANSparkMax ClimbMotor;

    public static RelativeEncoder ClimbMotorEncoder;
  
    public NewClimberSubsystem() {
        ClimbMotor = new CANSparkMax(40, MotorType.kBrushless);

        ClimbMotor.restoreFactoryDefaults();

        ClimbMotorEncoder = ClimbMotor.getEncoder();

        ClimbMotor.setInverted(true);

        ClimbMotor.setIdleMode(IdleMode.kBrake);

        ClimbMotor.setSmartCurrentLimit(40);

        ClimbMotor.setClosedLoopRampRate(2);

        ClimbMotor.burnFlash();
    }

    public void addDashboardWidgets(ShuffleboardLayout layout) {
        layout.withProperties(Map.of("Number of columns", 2, "Number of rows", 4));
        layout.addNumber("Left Conv factor", this::getLEFT_EncoderConvFactor)         .withPosition(0, 0);
        layout.addNumber("Left Position", this::getLeft_Position)                     .withPosition(0, 1);
        layout.addString("Climber Command", this::currentCommand)                     .withPosition(0, 2);
        layout.addString("Idle Mode", this::getIdleMode)                              .withPosition(0, 3);
        layout.addBoolean("Fast Lower", this::lowerClimberSpeed)                      .withPosition(1, 0);
        layout.addBoolean("Slower Raise", this::raiseClimberSpeed)                    .withPosition(1, 1);
    }

    public void lowerOVERRIDE() {
      if (fastLower) {
        ClimbMotor.set(fast_LOWER_SPEED);
      } else {
        ClimbMotor.set(LOWER_SPEED);
      }
    }

    public void raiseOVERRIDE() {
      if (fastLower) {
        ClimbMotor.set(fast_RAISE_SPEED);
      } else {
        ClimbMotor.set(RAISE_SPEED);
      }
    }

    public double getLEFT_EncoderConvFactor() {
        return ClimbMotorEncoder.getPositionConversionFactor();
    }

    public double getLeft_Position() {
        return ClimbMotorEncoder.getPosition();
    }

    public void setMotorPosition(double Position) {
        ClimbMotorEncoder.setPosition(Position);
    }

    public String getIdleMode() {
        if (ClimbMotor.getIdleMode().toString() == "kBrake") {
          return new String("Both Brk");
        } else  if (ClimbMotor.getIdleMode().toString() == "kBrake") {
          return new String("L=Brk & R=Cst");
        } else if (ClimbMotor.getIdleMode().toString() == "kCoast") {
          return new String("L=Cst & R=Brk");
        } else if (ClimbMotor.getIdleMode().toString() == "kCoast") {
          return new String("Both Cst");
        } else {
          return new String("err retreving");
        }
    }

    public boolean lowerClimberSpeed() {
        return fastLower;
    }

    public boolean raiseClimberSpeed() {
        return fastRaise;
    }

    public double getMotorVelocity() {
        return ClimbMotorEncoder.getVelocity();
    }

    public String currentCommand() {
        if (this.getCurrentCommand() != null) {
            return this.getCurrentCommand().getName();
        } else {
            return "none";
        }
    }

    public void setFastMovement() { 
      fastLower = true;
      fastRaise = true;
    }

    public void setSlowMovement() {
      fastLower = false;
      fastRaise = false;
    }

    public void setBrake() {
        ClimbMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setCoast() {
        ClimbMotor.setIdleMode(IdleMode.kCoast);
    }

    public void stopMotor() {
        ClimbMotor.stopMotor();
    } 

    public Command stopMotorCommand() {
        return run(() -> stopMotor());
    }

    public Command raiseMotorCommand() {
        return this.runEnd(
        () -> {
            raiseOVERRIDE();
        }, 
        () -> {
            stopMotor();
        }
        );
    }

    public Command raiseMotorCommand2() {
        return this.startEnd(        
        () -> {
            raiseOVERRIDE();
        }, 
        () -> {
            stopMotor();
        }
        );
    }

  public Command lowerMotorCommand() {
    return this.runEnd(
      () -> {
        lowerOVERRIDE();
      }, 
      () -> {
        stopMotor();
      }
    );
  }

  public Command setFastMovementCommand() {
    return run(() -> setFastMovement());
  }

  public Command setSlowMovemeCommand() {
    return run(() -> setSlowMovement());
  }

  @Override
  public void periodic() {
    /**if (fastLower) {
    LOWER_SPEED = -0.75;
    } else if (!fastLower) {
      LOWER_SPEED = -0.5;
    } else {
      LOWER_SPEED = -0.5;
    }

    if (fastRaise) {
      RAISE_SPEED = 0.75;
    } else if (!fastRaise) {
      RAISE_SPEED = 0.5;
    } else {
      RAISE_SPEED = 0.5;
    }*/
  } 

}
