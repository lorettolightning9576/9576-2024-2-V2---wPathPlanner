// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.Map;

import static frc.robot.Constants.IntakeConstants.leftIntakeMotorID;
import static frc.robot.Constants.IntakeConstants.rightIntakeMotorID;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  public static CANSparkMax leftIntakeMotor;
  public static CANSparkMax rightIntakeMotorLEADER;

  public static RelativeEncoder rightIntakeMotorEncoder;
  public static RelativeEncoder leftIntakeMotorEncoder;

  //public static SparkPIDController leftPID;
  //public static SparkPIDController rightPID;

  public static DigitalInput intakeNoteSensor;

  public IntakeSubsystem() {
    rightIntakeMotorLEADER =    new CANSparkMax(rightIntakeMotorID, MotorType.kBrushless);
    leftIntakeMotor =           new CANSparkMax(leftIntakeMotorID, MotorType.kBrushless);

    rightIntakeMotorLEADER.restoreFactoryDefaults();
    leftIntakeMotor.restoreFactoryDefaults();

    //leftPID = leftIntakeMotor.getPIDController();
    //rightPID = rightIntakeMotorLEADER.getPIDController();

    rightIntakeMotorEncoder = rightIntakeMotorLEADER.getEncoder();
    leftIntakeMotorEncoder = leftIntakeMotor.getEncoder();

    rightIntakeMotorLEADER.setIdleMode(IdleMode.kBrake);
    leftIntakeMotor.setIdleMode(IdleMode.kBrake);

    rightIntakeMotorLEADER.setInverted(true);
    leftIntakeMotor.setInverted(false);

    rightIntakeMotorLEADER.setSmartCurrentLimit(40);
    leftIntakeMotor.setSmartCurrentLimit(40);

    rightIntakeMotorLEADER.enableVoltageCompensation(12);
    leftIntakeMotor.enableVoltageCompensation(12);

    leftIntakeMotor.follow(rightIntakeMotorLEADER, true);

    /**double kP =          6.0e-5;
    double kI =          0;
    double kD =          0;
    double kIz =         0;
    double kFF =         0.000175;
    double kMaxOutput =  1.0;
    double kMinOutput = -1.0;

    leftPID.setP(kP);
    leftPID.setI(kI);
    leftPID.setD(kD);
    leftPID.setFF(kFF);
    leftPID.setOutputRange(kMinOutput, kMaxOutput);

    rightPID.setP(kP);
    rightPID.setI(kI);
    rightPID.setD(kD);
    rightPID.setFF(kFF);
    rightPID.setOutputRange(kMinOutput, kMaxOutput);*/

    rightIntakeMotorLEADER.burnFlash();
    leftIntakeMotor.burnFlash();
    
    intakeNoteSensor = new DigitalInput(0);
    
  }

  

  public boolean hasNote() {
    if (!intakeNoteSensor.get()) {
      return true;
    } else {
      return false;
    }
  }

  public boolean hasNoteRAW() {
    return intakeNoteSensor.get();
  }

  public void setIntakeIn() {
    rightIntakeMotorLEADER.set(0.375);
  }

  public void setIntakeFeed () {
    rightIntakeMotorLEADER.set(0.55);
  }

  public void setIntakeReverse() {
    rightIntakeMotorLEADER.set(Constants.IntakeConstants.intakeReverseSpeed);
  }

  public void stopIntake(){
    leftIntakeMotor.stopMotor();
    rightIntakeMotorLEADER.stopMotor();
  }

  public void setBrake() {
    leftIntakeMotor.setIdleMode(IdleMode.kBrake);
    rightIntakeMotorLEADER.setIdleMode(IdleMode.kBrake);
  }

  public void setIntakeCoast() {
    rightIntakeMotorLEADER.setIdleMode(IdleMode.kCoast);
    leftIntakeMotor.setIdleMode(IdleMode.kCoast);
  }

  public double getRightIntakeVelocity() {
    return rightIntakeMotorEncoder.getVelocity();
  }

  public double getLeftIntakeVelocity() {
    return rightIntakeMotorEncoder.getVelocity();
  }

  public String getIdleMode() {
    return rightIntakeMotorLEADER.getIdleMode().toString();
  }

  public String currentCommand() {
    if (this.getCurrentCommand() !=null) {
      return this.getCurrentCommand().getName();
    } else {
       return "none";
    }
  }

  public Command setIntakeFeedCommand() {
     return this.runEnd(
            ()-> {
                setIntakeFeed();
            },
            () -> {
                stopIntake();
        });
  }



  @Override
  public void periodic() {
    
  }

  public void addDashboardWidgets(ShuffleboardLayout layout) {
    layout.withProperties(Map.of("Number of columns", 2, "Number of rows", 4));
    layout.addNumber("Right Velocity", this::getRightIntakeVelocity)                    .withPosition(0, 0);
    layout.addNumber("Left Velocity", this::getLeftIntakeVelocity)                      .withPosition(0, 1);
    layout.addBoolean("Has Note", this::hasNoteRAW)                                        .withPosition(0, 2);
    layout.addString("Right Idle Mode", this::getIdleMode)                              .withPosition(0, 3);
    layout.addString("Current Command", this::currentCommand)                           .withPosition(1, 0);
}
}
