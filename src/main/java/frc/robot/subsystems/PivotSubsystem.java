package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAnalogSensor.Mode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;
import static com.revrobotics.SparkPIDController.ArbFFUnits.kVoltage;

import static frc.robot.Constants.IOSwitchPorts.pivotDownLimitID;
import static frc.robot.Constants.IOSwitchPorts.pivotUpLimitID;

import static com.revrobotics.SparkLimitSwitch.Type.kNormallyOpen;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

public class PivotSubsystem extends SubsystemBase{
  private double targetAngle;
  /**
   * Theoretically if you were off when you install the hex shaft,
   * it would change in 60 degree intervals
   */
  //private static Measure<Angle> PivotOffset = Degrees.of(1);

  public static double PotentiometerOffset = 0.0;
  public static final double Soft_Limit_Foward = 6.75; // Rotations
  public static final double Soft_Limit_Reverse = 1.3; //Rotations
  public static final double Range_Of_Motion = (1.0 / (130 / 10) * 10);
  //                                      (360°/(130 teeth/10 teeth))*10 turns = 276.92 Degrees of movement on the gear rack
  public static final double factor = (1.0 / (130.0 / 10.0));
  public static final double factorTooDegrees = (factor * 360);
  //public static final double factorV2 = (5.0 / (1.0 / (130.0 / 10.0) * 10.0));

  /**
   *Rotations
   */
  private static final float LIMIT_UP = 5.0f;
  /**
    *Rotations
    */
  private static final float LIMIT_DOWN = 0.0f;

  private static final Measure<Voltage> Gravity_Feed_Forward = Volts.of(0.2);
  public double aimTargetRotations = 0.0;
  private boolean holdAimPosition = false;

  public static Measure<Angle> LowAim_Error_Tolerance = Rotations.of(0.525);
  public static Measure<Angle> HighAim_Error_Tolerance = Rotations.of(0.575);

  private DigitalInput pivotDownSwitch;
  private DigitalInput pivotUpSwitch;

  public final CANSparkMax leftPivotMotor;
  public final CANSparkMax rightPivotMotorLEADER;

  public RelativeEncoder leftMotorEncoder;
  public RelativeEncoder rightMotorLEADEREncoder;

  private final SparkPIDController pivotPidController;
  public final SparkAnalogSensor potentiometer;

  private CommandXboxController commandXboxController;

  public PivotSubsystem(CommandXboxController m_commandXboxController) {
    leftPivotMotor = new CANSparkMax(20, MotorType.kBrushless);
    rightPivotMotorLEADER = new CANSparkMax(21, MotorType.kBrushless);

    leftPivotMotor.restoreFactoryDefaults();
    rightPivotMotorLEADER.restoreFactoryDefaults();

    rightMotorLEADEREncoder = rightPivotMotorLEADER.getEncoder();
    leftMotorEncoder = leftPivotMotor.getEncoder();

    pivotPidController = rightPivotMotorLEADER.getPIDController(); 

    var externalpot = rightPivotMotorLEADER.getAnalog(Mode.kAbsolute);
    potentiometer = externalpot;

    //externalpot.setPositionConversionFactor(factor);
    externalpot.setPositionConversionFactor(4.357);
    pivotPidController.setFeedbackDevice(externalpot);


    double kP =  0.4;
    double kI =  0;
    double kD =  0.0000;
    double kIz = 0;
    double kFF = 0;
    double kMaxOutput =  0.5;
    double kMinOutput = -0.5;

    pivotPidController.setP(kP);
    pivotPidController.setI(kI);
    pivotPidController.setD(kD);
    pivotPidController.setIZone(kIz);
    pivotPidController.setFF(kFF);
    pivotPidController.setOutputRange(kMinOutput, kMaxOutput); 
    pivotPidController.setPositionPIDWrappingMaxInput(13.0);
    pivotPidController.setPositionPIDWrappingMinInput(0);
    pivotPidController.setPositionPIDWrappingEnabled(true);

    rightPivotMotorLEADER.enableVoltageCompensation(12);
    leftPivotMotor.enableVoltageCompensation(12);

    rightPivotMotorLEADER.setSoftLimit(SoftLimitDirection.kForward, LIMIT_UP);
    rightPivotMotorLEADER.setSoftLimit(SoftLimitDirection.kReverse, LIMIT_DOWN);
    rightPivotMotorLEADER.enableSoftLimit(SoftLimitDirection.kForward, false);
    rightPivotMotorLEADER.enableSoftLimit(SoftLimitDirection.kReverse, false);

    rightPivotMotorLEADER.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false);
    rightPivotMotorLEADER.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false);

    rightPivotMotorLEADER.setIdleMode(IdleMode.kBrake);
    leftPivotMotor.setIdleMode(IdleMode.kBrake); 

    rightPivotMotorLEADER.setInverted(true);
    leftPivotMotor.setInverted(false);  

    leftPivotMotor.follow(rightPivotMotorLEADER, true);  

    leftPivotMotor.setSmartCurrentLimit(40);
    rightPivotMotorLEADER.setSmartCurrentLimit(40);  

    leftPivotMotor.burnFlash();
    rightPivotMotorLEADER.burnFlash();

    pivotDownSwitch =   new DigitalInput(pivotDownLimitID);
    pivotUpSwitch =     new DigitalInput(pivotUpLimitID);

    this.commandXboxController = m_commandXboxController;
  }

  public void addDashboardWidgets(ShuffleboardLayout layout) {
    layout.withProperties(Map.of("Number of columns", 4, "Number of rows", 4));
    layout.addNumber("Rotations to Radians", this::getPivotPosition)                 .withPosition(0, 0);
    layout.addNumber("Rotations to Degrees", this::getPivotAngle)                    .withPosition(0, 1);
    //layout.addNumber("Position Raw", pivotAbsoluteEncoder::getPosition)                   .withPosition(0, 2);
    layout.addNumber("Position Raw", potentiometer::getPosition)                    .withPosition(0, 2);
    layout.addNumber("Pivot Offset", this::getPivotOffset)                          .withPosition(0, 3);
    layout.addNumber("Target Position", this::getGoalPosition)                      .withPosition(1, 0);
    layout.addNumber("Pot Offset in Rotations", this::PotOffsetInRot)               .withPosition(1, 1);
    //layout.addNumber("Pivot Tolerance Factor", this::pivotToleranceFactor)                .withPosition(1, 1);
    layout.addString("Pivot Command", this::currentCommand)                         .withPosition(1, 2);
    layout.addNumber("Raw - Offset", this::getPivotRawMinusOffset)                  .withPosition(1, 3);
    //layout.addNumber("What it sees", this::whatThePIDsees)                                .withPosition(1, 3);
    layout.addBoolean("Is Within Tolerance", this::isAimAtTargetPosition)           .withPosition(2, 3);
    layout.addNumber("Left Motor Velocity", leftMotorEncoder::getVelocity)          .withPosition(2, 0);
    layout.addNumber("Right Motor Veocity", rightMotorLEADEREncoder::getVelocity)   .withPosition(2, 1); 
    layout.addNumber("Degrees Devide Thirteen", this::getPivotAngleDevideBy13)      .withPosition(2, 2);
    //layout.addNumber("PID sees", this::getPivotPositionMinusOffset)                 .withPosition(2, 3);
    //layout.addNumber("High Tolerance", this::getHighTolerance)                            .withPosition(2, 2);
    //layout.addNumber("Low Tolerance", this::getLowTolerance)                              .withPosition(2, 3);
    layout.addNumber("Position Conv Factor", this::getConvFactor)                   .withPosition(3, 0);
    //layout.addNumber("Encoder Velocity", this::getEncoderVelocity)                        .withPosition(3, 2);
    layout.addBoolean("Down Switch Status", this::downSwitchStatus)                 .withPosition(3, 1);
    layout.addBoolean("Up Switch Status", this::upSwitchStatus)                     .withPosition(3, 2);
    layout.addString("Left Idle Mode", this::getRightIdleMode)                      .withPosition(3, 3);
  } 

  @Override
  public void periodic() {
    /**if (this.getCurrentCommand() != null) {
      SmartDashboard.putString("Pivot Command", this.getCurrentCommand().getName());
    } else {
      SmartDashboard.putString("Pivot Command", "none");
    }*/
    
    /**if (holdAimPosition) {
      double cosineScalar = Math.cos(getPivotPosition());
      double feedFoward = Gravity_Feed_Forward.in(Volts) * cosineScalar;

        pivotPidController.setReference(
          aimTargetRotations, 
          ControlType.kPosition, 
          0, 
          feedFoward, 
          kVoltage
        );
    }*/

    if (holdAimPosition) {
        double cosineScalar = Math.cos(getPivotPositionMinusOffset());
        double feedFoward = Gravity_Feed_Forward.in(Volts) * cosineScalar;

      if (!pivotDownSwitch.get() && !pivotUpSwitch.get()) {
        pivotPidController.setReference(
          aimTargetRotations, 
          ControlType.kPosition, 
          0, 
          feedFoward, 
          kVoltage
        );
      } else if (pivotDownSwitch.get() && aimTargetRotations < Soft_Limit_Foward) {
        pivotPidController.setReference(
          aimTargetRotations, 
          ControlType.kPosition, 
          0, 
          feedFoward, 
          kVoltage
        );
      } else if (pivotUpSwitch.get() && aimTargetRotations > Soft_Limit_Reverse) {
        pivotPidController.setReference(
          aimTargetRotations, 
          ControlType.kPosition, 
          0, 
          feedFoward, 
          kVoltage
        );
      } else {
        stopAimAndMotors();
      }
    }
  }

  public double getAimTargetRotations_wOffset() {
    return aimTargetRotations + PotentiometerOffset;
  }

  /**public static double degPosToNativePotentiometer(double degrees) {
    return(degrees - PotentiometerOffset) / 
  }*/

  public void setArmTriggerSpeed() {
    holdAimPosition = false;
    if (pivotDownSwitch.get()) {
      if (commandXboxController.getRawAxis(1) > 0) {
        stopAimAndMotors();
      } else if (commandXboxController.getRawAxis(1) < 0) {
        rightPivotMotorLEADER.set(MathUtil.applyDeadband(commandXboxController.getRawAxis(1) * 0.375, 0.1));
      } else {
        stopAimAndMotors();
      }
    } else if (!pivotUpSwitch.get()) {
      if (commandXboxController.getRawAxis(1) < 0) {
        stopAimAndMotors();
      } else if (commandXboxController.getRawAxis(1) > 0) {
        rightPivotMotorLEADER.set(MathUtil.applyDeadband(commandXboxController.getRawAxis(1) * 0.375, 0.1));
      }  else {
        stopAimAndMotors();
      }
    } else if (pivotUpSwitch.get() && !pivotDownSwitch.get()) {
      rightPivotMotorLEADER.set(MathUtil.applyDeadband(commandXboxController.getRawAxis(1) * 0.375, 0.1));
    } else {
      stopAimAndMotors();
    }
  }

  public void setCoast() {
    rightPivotMotorLEADER.setIdleMode(IdleMode.kCoast);
    leftPivotMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setBrake() {
    rightPivotMotorLEADER.setIdleMode(IdleMode.kBrake);
    leftPivotMotor.setIdleMode(IdleMode.kBrake);
  }

  /**public double getPivotPosition() {
    return Units.rotationsToRadians(potentiometer.getPosition()  - PotentiometerOffset);
  }*/

  public double getPivotPosition() {
    return Units.rotationsToRadians(potentiometer.getPosition());
  }

  public double getPivotPositionMinusOffset() {
    return Units.rotationsToRadians(potentiometer.getPosition() + PotentiometerOffset);
  }

  public double getPivotRawMinusOffset() {
    return potentiometer.getPosition()  - PotentiometerOffset;
  }

  /**public double PotOffsetInRot() {
    return PotentiometerOffset.in(Rotations);
  }*/

  public double PotOffsetInRot() {
    return PotentiometerOffset;
  }

  /**public double getPivotAngle() {
    return Units.rotationsToDegrees(analogPotentiometer.get());
  }*/

  public double getConvFactor() {
    return rightMotorLEADEREncoder.getPositionConversionFactor();
  }

  /**public double whatThePIDsees() {
    return potentiometer.getPosition() - PotentiometerOffset.in(Rotations);
  }*/

  /**public double getPivotAngle() {
    return Units.rotationsToDegrees(potentiometer.getPosition() - PotentiometerOffset);
  }*/
  
  public double getPivotAngle() {
    return Units.rotationsToDegrees(potentiometer.getPosition());
  }

  public double getPivotAngleDevideBy13() {
    return (Units.rotationsToDegrees(potentiometer.getPosition() - PotentiometerOffset)/ 13);
  }

  /**public void setEncoderOffset() {
    PivotOffset = Rotations.of(pivotAbsoluteEncoder.getPosition());
    
    pivotAbsoluteEncoder.setZeroOffset(PivotOffset.in(Rotations));
  }


  public double getPivotPosition() {
    return Units.rotationsToRadians(pivotAbsoluteEncoder.getPosition());
  }

  public double getPivotAngle() {
    return Units.rotationsToDegrees(pivotAbsoluteEncoder.getPosition());
  }

  public double getPivotAngleCONVERTED() {
    return Units.rotationsToDegrees(pivotAbsoluteEncoder.getPosition() / 13);
  }

  public double getPivotPositionConveted() {
    return Units.rotationsToDegrees(pivotAbsoluteEncoder.getPosition());
  }*/

  public double getPivotOffset() {
    return PotentiometerOffset;
  }

  static double pivotRadiansToEncoderRotations(double pivotRadians) {
    return Units.radiansToRotations(pivotRadians);
  }

  /**public double getPivotVelocity() {
    return Units.rotationsToDegrees(pivotAbsoluteEncoder.getVelocity());
  }*/

  public boolean isAimAtTargetPosition() {
    return (Math.abs(Units.rotationsToDegrees(potentiometer.getPosition())) < (Units.rotationsToDegrees(aimTargetRotations) + 20)) && 
           (Math.abs(Units.rotationsToDegrees(potentiometer.getPosition())) > (Units.rotationsToDegrees(aimTargetRotations) - 20));
    
  }

  public boolean isTooLow() {
    return(Math.abs(potentiometer.getPosition()) > 5.775);
  }

  private boolean downSwitchStatus() {
    return pivotDownSwitch.get();
  }

  private boolean upSwitchStatus() {
    return pivotUpSwitch.get();
  }

  /**private double pivotToleranceFactor() {
    return Math.abs(pivotAbsoluteEncoder.getPosition() - aimTargetRotations);
  }*/

  public String currentCommand() {
    if (this.getCurrentCommand() != null) {
      return this.getCurrentCommand().getName();
    } else {
      return "none";
    }
  }

  public String getRightIdleMode() {
    return rightPivotMotorLEADER.getIdleMode().toString();
  }

  public double getGoalPosition() {
    return aimTargetRotations;
  }

  public void setPivotPosition(double angle) {
    targetAngle = (angle);
    holdAimPosition = true;
    aimTargetRotations = targetAngle;

    HighAim_Error_Tolerance = Rotations.of(aimTargetRotations + 0.1);
    LowAim_Error_Tolerance = Rotations.of(aimTargetRotations - 0.1);
  }

  public void stopAimAndMotors() {
    holdAimPosition = false;
    rightPivotMotorLEADER.stopMotor();
    leftPivotMotor.stopMotor();
  }

  public double getHighTolerance() {
    return HighAim_Error_Tolerance.in(Rotations);
  }

  public double getLowTolerance() {
    return LowAim_Error_Tolerance.in(Rotations);
  }

  /**public double getConvFactor() {
    return potentiometer.getPositionConversionFactor();
  }*/

  /**public double getEncoderVelocity() {
    return potentiometer.getVelocity();
  }*/

  /**public double getConvFactor() {
    return pivotAbsoluteEncoder.getPositionConversionFactor();
  }

  public double getEncoderVelocity() {
    return pivotAbsoluteEncoder.getVelocity();
  }*/

  public void setPivotShootUnderSpeaker() {
    double  ShootSpeaker = 4.755;
    setPivotPosition(ShootSpeaker);
  }

  public void setPivotShootStage() {
    double  ShootStage = 5.71;
    setPivotPosition(ShootStage);
  }

    public void setPivotSHUTTLE() {
    double  Shuttle = 5.05;
    setPivotPosition(Shuttle);
  }

  public void front3notesAuto() {
    double  ShootStage = 5.65;
    setPivotPosition(ShootStage);
  }

  public void setPivotIntake() {
    double  Intake = 5.71;
    setPivotPosition(Intake);
  }

  /**public void setPivotStart_AMP() {
    double  Start_AMP = 2.2;
    setPivotPosition(Start_AMP);
  }*/

  public void setPivotFinish_AMP() {
    double  Finish_AMP = 1.7;
    setPivotPosition(Finish_AMP);
  }

  public void setPivot_StraightUp() {
    double  Finish_AMP = 3.20;
    setPivotPosition(Finish_AMP);
  }

  public Command setPivotIntakeCommand() {
    return run(() -> setPivotIntake());
  }

  /**public Command setPivotStart_AMPCommand() {
    return run(() -> setPivotStart_AMP());
  }*/

  public Command setPivot_Finish_AMPCommand() {
    return run(() -> setPivotFinish_AMP());
  }

  public Command setPivot_Vertical_Command() {
    return run(() -> setPivot_StraightUp());
  }

  public Command setPivot_Shuttle_Command() {
    return run(() -> setPivotSHUTTLE());
  }

  public Command setPivotShootSpeakerCommand() {
    return run(() -> setPivotShootUnderSpeaker());
  }

  public Command setPivotShootStageCommand() {
    return run(() -> setPivotShootStage());
  }

    public Command autoFront3Notes_Command() {
    return run(() -> front3notesAuto());
  }
}
