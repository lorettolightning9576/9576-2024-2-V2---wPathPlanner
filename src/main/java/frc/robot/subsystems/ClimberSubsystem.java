// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.Map;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private double raiseLimit = 210.0;

  private double RAISE_SPEED = 0.75;
  private double LOWER_SPEED = -0.25;

  public boolean fastLower = false;
  public boolean slowRaise = false;

  public static CANSparkMax leftClimbMotor;
  public static CANSparkMax rightClimbMotor;

  public static RelativeEncoder leftClimbMotorEncoder;
  public static RelativeEncoder rightClimbMotorEncoder;

  public static DigitalInput leftClimberSwitch;
  public static DigitalInput rightClimberSwitch;
  public ClimberSubsystem() {
    leftClimbMotor = new CANSparkMax(40, MotorType.kBrushless);
    rightClimbMotor = new CANSparkMax(41, MotorType.kBrushless);

    leftClimbMotor.restoreFactoryDefaults();
    rightClimbMotor.restoreFactoryDefaults();

    leftClimbMotorEncoder = leftClimbMotor.getEncoder();
    rightClimbMotorEncoder = rightClimbMotor.getEncoder();

    leftClimbMotor.setInverted(true);
    rightClimbMotor.setInverted(false);

    leftClimbMotor.setIdleMode(IdleMode.kBrake);
    rightClimbMotor.setIdleMode(IdleMode.kBrake);

    leftClimbMotor.setSmartCurrentLimit(40);
    rightClimbMotor.setSmartCurrentLimit(40);

    leftClimbMotor.setClosedLoopRampRate(2);
    rightClimbMotor.setClosedLoopRampRate(2);

    leftClimbMotor.burnFlash();
    rightClimbMotor.burnFlash();

    leftClimberSwitch = new DigitalInput(5);
    rightClimberSwitch = new DigitalInput(6);
  }

  public void addDashboardWidgets(ShuffleboardLayout layout) {
    layout.withProperties(Map.of("Number of columns", 3, "Number of rows", 4));
    layout.addNumber("Left Conv factor", this::getLEFT_EncoderConvFactor)         .withPosition(0, 0);
    layout.addNumber("Right Conv Factor", this::getRIGHT_EncoderConvFactor)       .withPosition(0, 1);
    layout.addNumber("Right Positon", this::getRIGHT_Position)                    .withPosition(0, 2);
    layout.addNumber("Left Position", this::getLeft_Position)                     .withPosition(0, 3);
    layout.addString("Climber Command", this::currentCommand)                     .withPosition(1, 0);
    layout.addBoolean("Left Switch", this::leftSwitchGet)                         .withPosition(1, 1);
    layout.addBoolean("Right Switch", this::rightSwitchGet)                       .withPosition(1, 2);
    layout.addString("Idle Mode", this::getIdleMode)                              .withPosition(1, 3);
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

  public void raiseBothArm() {
    if (leftClimbMotorEncoder.getPosition() < raiseLimit && rightClimbMotorEncoder.getPosition() < raiseLimit) {
      leftClimbMotor.set(RAISE_SPEED); 
      rightClimbMotor.set(RAISE_SPEED);
    } else if (rightClimbMotorEncoder.getPosition() < raiseLimit) {
      rightClimbMotor.set(RAISE_SPEED);
    } else if (leftClimbMotorEncoder.getPosition() < raiseLimit) {
      leftClimbMotor.set(RAISE_SPEED); 
    } else {
      stopBothArms();
    }
  }

  public void lowerLeftArm() {
    if (!leftClimberSwitch.get()) {
      leftClimbMotor.set(LOWER_SPEED);  
    } else {
      leftClimbMotor.stopMotor();
    }
  }

  public void lowerLeftArmOVERRIDE() {
    leftClimbMotor.set(LOWER_SPEED); 
  }

  public void lowerRightArmOVERRIDE() {
    rightClimbMotor.set(LOWER_SPEED);
  }

  public void lowerRightArm() {
    if (!rightClimberSwitch.get()) {
      rightClimbMotor.set(LOWER_SPEED);
    } else {
      rightClimbMotor.stopMotor();
    }
  }

  public void lowerBothArm() {
  lowerLeftArm();
  lowerRightArm();
  }

  public void raiseRightArm() {
    if (rightClimbMotorEncoder.getPosition() < raiseLimit) {
      rightClimbMotor.set(RAISE_SPEED); 
    } else {
      stopRightMotor();
    }
  }

  public double getLEFT_EncoderConvFactor() {
    return leftClimbMotorEncoder.getPositionConversionFactor();
  }

  public double getRIGHT_EncoderConvFactor() {
    return rightClimbMotorEncoder.getPositionConversionFactor();
  }

  public double getLeft_Position() {
    return leftClimbMotorEncoder.getPosition();
  }

  public double getRIGHT_Position() {
    return rightClimbMotorEncoder.getPosition();
  }

  public void setLEFT_Position() {
    leftClimbMotorEncoder.setPosition(0.1);
  }

  public String getIdleMode() {
    if (leftClimbMotor.getIdleMode().toString() == "kBrake" && rightClimbMotor.getIdleMode().toString() == "kBrake") {
      return new String("Both Brk");
    } else  if (leftClimbMotor.getIdleMode().toString() == "kBrake" && rightClimbMotor.getIdleMode().toString() == "kCoast") {
      return new String("L=Brk & R=Cst");
    } else if (leftClimbMotor.getIdleMode().toString() == "kCoast" && rightClimbMotor.getIdleMode().toString() == "kBrake") {
      return new String("L=Cst & R=Brk");
    } else if (leftClimbMotor.getIdleMode().toString() == "kCoast" && rightClimbMotor.getIdleMode().toString() == "kCoast") {
      return new String("Both Cst");
    } else {
      return new String("err retreving");
    }
  }

  public boolean LowerClimberSpeed() {
    return fastLower;
  }

  public boolean raiseClimberSpeed() {
    return slowRaise;
  }

  public boolean leftSwitchGet() {
    return leftClimberSwitch.get();
  }

  public boolean rightSwitchGet() {
    return rightClimberSwitch.get();
  }

  public void setRIGHT_Position() {
    rightClimbMotorEncoder.setPosition(0.1);
  }

  public double getRightVeolcity() {
    return rightClimbMotorEncoder.getVelocity();
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
    rightClimbMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
    leftClimbMotor.setIdleMode(IdleMode.kCoast);
    rightClimbMotor.setIdleMode(IdleMode.kCoast);
  }

  public void stopLeftMotor() {
    leftClimbMotor.stopMotor();
  } 

  public void stopRightMotor() {
    rightClimbMotor.stopMotor();
  }

  public void stopBothArms() {
    stopLeftMotor();
    stopRightMotor();
  }

  public Command stopBothArmsCommand() {
    return run(() -> stopBothArms());
  }

  public Command raiseRightArmCommand() {
    return this.runEnd(
      () -> {
        raiseRightArm();
      }, 
      () -> {
        stopRightMotor();
      }
    );
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

  public Command lower_LEFT_ArmCommand() {
    return this.runEnd(
      () -> {
        lowerLeftArm();
      }, 
      () -> {
        stopLeftMotor();
      }
    );
  }

  public Command lower_RIGHT_ArmCommand() {
    return this.runEnd(
      () -> {
        lowerRightArm();
      }, 
      () -> {
        stopRightMotor();
      }
    );
  }


  public Command lower_BOTH_ArmCommand() {
    return this.runEnd(
      () -> {
        lowerBothArm();
      }, 
      () -> {
        stopBothArms();
      }
    );
  }

  public Command raise_BOTH_ArmCommand() {
    return this.runEnd(
      () -> {
        raiseBothArm();
      }, 
      () -> {
        stopBothArms();
      }
    );
  }

  @Override
  public void periodic() {
    if (leftClimberSwitch.get()) {
      setLEFT_Position();
    }
    
    if (rightClimberSwitch.get()) {
      setRIGHT_Position();
    }

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