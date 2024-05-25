// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private double raiseLimit = 210.0;

  private double RAISE_SPEED = 0.9;
  private double LOWER_SPEED = -0.45;

  public boolean fastLower = false;
  public boolean slowRaise = false;

  public static CANSparkMax leftClimbMotor;

  public static RelativeEncoder leftClimbMotorEncoder;

  public ClimberSubsystem() {
    leftClimbMotor = new CANSparkMax(40, MotorType.kBrushless);

    leftClimbMotor.restoreFactoryDefaults();

    leftClimbMotorEncoder = leftClimbMotor.getEncoder();

    leftClimbMotor.setInverted(true);

    leftClimbMotor.setIdleMode(IdleMode.kBrake);

    leftClimbMotor.setSmartCurrentLimit(40);

    leftClimbMotor.burnFlash();
  }

  public void addDashboardWidgets(ShuffleboardLayout layout) {
    layout.withProperties(Map.of("Number of columns", 3, "Number of rows", 4));
    layout.addNumber("Left Conv factor", this::getLEFT_EncoderConvFactor)         .withPosition(0, 0);
    layout.addNumber("Left Position", this::getLeft_Position)                     .withPosition(0, 3);
    layout.addString("Climber Command", this::currentCommand)                     .withPosition(1, 0);
    layout.addBoolean("Fast Lower", this::LowerClimberSpeed)                      .withPosition(2, 0);
    layout.addBoolean("Slower Raise", this::raiseClimberSpeed)                    .withPosition(2, 1);
  }

  public void raiseLeftArm() {
    if (leftClimbMotorEncoder.getPosition() < raiseLimit) {
      leftClimbMotor.set(RAISE_SPEED); 
    } else {
      stopLeftMotor();
    }
  }

  /**public void lowerLeftArm() {
    if (!leftClimberSwitch.get()) {
      leftClimbMotor.set(LOWER_SPEED);  
    } else {
      leftClimbMotor.stopMotor();
    }
  }*/

  public void lowerLeftArmOVERRIDE() {
    leftClimbMotor.set(LOWER_SPEED); 
  }

  public void raiseLeftArmOVERRIDE() {
    leftClimbMotor.set(-LOWER_SPEED); 
  }

  public Command raiseLeft() {
    return run(() -> raiseLeftArmOVERRIDE());
  }

  public Command lowerLeft() {
    return run(() -> lowerLeftArmOVERRIDE());
  }

  public double getLEFT_EncoderConvFactor() {
    return leftClimbMotorEncoder.getPositionConversionFactor();
  }

  public double getLeft_Position() {
    return leftClimbMotorEncoder.getPosition();
  }

  public void setLEFT_Position() {
    leftClimbMotorEncoder.setPosition(0.1);
  }

  public boolean LowerClimberSpeed() {
    return fastLower;
  }

  public boolean raiseClimberSpeed() {
    return slowRaise;
  }

  public double getLeftVelocity() {
    return leftClimbMotorEncoder.getVelocity();
  }

  public String currentCommand() {
    if (this.getCurrentCommand() != null) {
        return this.getCurrentCommand().getName();
    } else {
        return "none";
    }
  }

  public void setBrake() {
    leftClimbMotor.setIdleMode(IdleMode.kBrake); 
  }

  public void setCoast() {
    leftClimbMotor.setIdleMode(IdleMode.kCoast);
  }

  public void stopLeftMotor() {
    leftClimbMotor.stopMotor();
  } 

  public Command stopLeft() {
    return run(() -> stopLeftMotor());
  }

  public void stopBothArms() {
    stopLeftMotor();
  }

  public Command stopBothArmsCommand() {
    return run(() -> stopBothArms());
  }

  public Command raiseLeftArmCommand() {
    return this.runEnd(
      () -> {
        raiseLeftArm();
      }, 
      () -> {
        stopLeftMotor();
      }
    );
  }

  @Override
  public void periodic() {

    if (fastLower) {
      LOWER_SPEED = -0.95;
    } else if (!fastLower) {
      LOWER_SPEED = -0.35;
    } else {
      LOWER_SPEED = -0.35;
    }

    if (slowRaise) {
      RAISE_SPEED = 0.5;
    } else if (!slowRaise) {
      RAISE_SPEED = 0.95;
    } else {
      RAISE_SPEED = 0.95;
    }
  }
}