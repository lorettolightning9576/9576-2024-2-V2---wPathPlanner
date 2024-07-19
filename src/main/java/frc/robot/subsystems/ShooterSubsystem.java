package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Map;

public class ShooterSubsystem extends SubsystemBase{

    private double TopTargetVelocity;
    private double BottomTargetVelocity;

    private double TopTargetSpeed;
    private double BottomTargetSpeed;

    private static boolean setShooterVelocity;
    private double shooterSpeed;

    public double shooterRampRate = 1.5;

    public static CANSparkMax shooterBottomMotor;
    public static CANSparkMax shooterTopMotor;

    private SparkPIDController topPID;
    private SparkPIDController bottomPID;

    public RelativeEncoder shooterBottomMotorEncoder;
    public RelativeEncoder shooterTopMotorEncoder;

    public ShooterSubsystem() {
        shooterBottomMotor = new CANSparkMax(Constants.ShooterConstants.bottomShooterMotorID, MotorType.kBrushless);
        shooterBottomMotor.restoreFactoryDefaults();
        bottomPID = shooterBottomMotor.getPIDController();
        shooterBottomMotorEncoder = shooterBottomMotor.getEncoder();
        shooterBottomMotorEncoder.setVelocityConversionFactor(1);
        shooterBottomMotor.setIdleMode(IdleMode.kCoast);
        shooterBottomMotor.setInverted(false);
        shooterBottomMotor.setSmartCurrentLimit(40);
        shooterBottomMotor.enableVoltageCompensation(12);

        double kP_Bottom =          6.0e-5;
        double kI_Bottom =          0;
        double kD_Bottom =          0;
        double kFF_Bottom =         0.0001725;
        double kMaxOutput_Bottom =  1.0;
        double kMinOutput_Bottom = -1.0;

        bottomPID.setP(kP_Bottom);
        bottomPID.setI(kI_Bottom);
        bottomPID.setD(kD_Bottom);
        bottomPID.setFF(kFF_Bottom);
        bottomPID.setOutputRange(kMinOutput_Bottom, kMaxOutput_Bottom);
        shooterBottomMotor.burnFlash();

        shooterTopMotor = new CANSparkMax(Constants.ShooterConstants.topShooterMotorID, MotorType.kBrushless);
        shooterTopMotor.restoreFactoryDefaults();
        topPID = shooterTopMotor.getPIDController();
        shooterTopMotor.setIdleMode(IdleMode.kCoast);
        shooterTopMotor.setInverted(false);
        shooterTopMotorEncoder = shooterTopMotor.getEncoder();
        shooterTopMotorEncoder.setVelocityConversionFactor(1);
        shooterTopMotor.setSmartCurrentLimit(40);
        shooterTopMotor.enableVoltageCompensation(12);

        topPID.setP(kP_Bottom);
        topPID.setI(kI_Bottom);
        topPID.setD(kD_Bottom);
        topPID.setFF(kFF_Bottom);
        topPID.setOutputRange(kMinOutput_Bottom, kMaxOutput_Bottom);
        shooterTopMotor.burnFlash();

    }

    public void addDashboardWidgets(ShuffleboardLayout layout) {
        layout.withProperties(Map.of("Number of columns", 3, "Number of rows", 4));
        layout.addNumber("Bottom Velocity", this::getBottomVelocity)                .withPosition(0, 0);
        layout.addNumber("Top Velocity", this::getTopVelocity)                      .withPosition(0, 1);
        layout.addString("Shooter Command", this::getcurrentCommandS)               .withPosition(0, 2);
        layout.addNumber("Top Target Velocity", this::getTopTargetVelocity)         .withPosition(0, 3);
        layout.addNumber("Bottom Target Velocity", this::getBottomTargetVelocity)   .withPosition(1, 0);
        layout.addString("Top Idle Mode", this::getTopIdleMode)                     .withPosition(1, 1);
        layout.addString("Bottom Idle Mode", this::getBottomIdleMode)               .withPosition(1, 2);
        layout.addNumber("Bottom Roller Speed", this::getBottomRollerSpeed)         .withPosition(2, 0);
        layout.addNumber("Top Roller Speed", this::getTopRollerSpeed)               .withPosition(2, 1);
        layout.addBoolean("Is at Target Velocity Tolerance", this::isAtTargetVelocity)    .withPosition(2, 2);
        layout.addBoolean("Is at Target Speed Tolerance", this::isAtTargetSpeed)    .withPosition(2, 3);
    }

    public void setShooterCustomVelocity(double velocity) {
        BottomTargetVelocity = velocity;
        TopTargetVelocity = velocity;

        bottomPID.setReference(BottomTargetVelocity, ControlType.kVelocity);
        topPID.setReference(TopTargetVelocity, ControlType.kVelocity);
    }

    public void setShootVelociy() {
        BottomTargetVelocity = 4500;
        TopTargetVelocity = 4500;
        setShooterVelocity = true;
    }

    public Command setShoot1500Command() {
        BottomTargetVelocity = 1500;
        TopTargetVelocity = 1500;

        return this.runEnd(
            () -> {
                bottomPID.setReference(BottomTargetVelocity, ControlType.kVelocity);
                topPID.setReference(TopTargetVelocity, ControlType.kVelocity);
            }, 
            () -> {
                stopShooter();
            }
        );
    }

    public void setShooterShoot () {
        TopTargetVelocity = 4500;
        BottomTargetVelocity = 4480;
        setShooterVelocity = true;
    }

    public void setShooterSHUTTLE() {
        TopTargetVelocity = 4000;
        BottomTargetVelocity = 4000;
        TopTargetVelocity = 4300;
        BottomTargetVelocity = 4280;
        setShooterVelocity = true;
    }

    public void setShooterAmpShot() {
        TopTargetVelocity = 500;
        BottomTargetVelocity = 500;
        setShooterVelocity = true;
    }

    public void setShooterAMPSpeed() {
        setShooterVelocity = false;

        BottomTargetSpeed = 0.09;
        TopTargetSpeed = 0.3;

        shooterTopMotor.set(TopTargetSpeed);
        shooterBottomMotor.set(BottomTargetSpeed);
    }

    public void setShooterAmp () {
        setShooterCustomVelocity(Constants.ShooterConstants.shooterAmpVelocity);
    }

    public void setShooterIn () {
        setShooterCustomVelocity(Constants.ShooterConstants.shooterInVelocity);
    }

    public void setShooterCustomSpeed(double speed) {
        shooterSpeed = speed;
        shooterTopMotor.set(shooterSpeed);
        shooterBottomMotor.set(shooterSpeed);
    }

    public void stopShooter() {
        setShooterVelocity = false;
        shooterBottomMotor.stopMotor();
        shooterTopMotor.stopMotor();
    }

    // Sets The Shooter Motors to Coast
    public void setShooterCoast() {
        shooterBottomMotor.setIdleMode(IdleMode.kCoast);
        shooterTopMotor.setIdleMode(IdleMode.kCoast);
    }

    // Sets Shooter Motors to Brake
    public void setShooterBrake() {
        shooterTopMotor.setIdleMode(IdleMode.kBrake);
        shooterBottomMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        if (shooterTopMotorEncoder.getVelocity() < 0) {
            SmartDashboard.putNumber("Top Shooter Velocity", shooterTopMotorEncoder.getVelocity() * -1);
        } else {
            SmartDashboard.putNumber("Top Shooter Velocity", shooterTopMotorEncoder.getVelocity());
        }

        if (shooterBottomMotorEncoder.getVelocity() < 0) {
            SmartDashboard.putNumber("Bottom Shooter Velocity", shooterBottomMotorEncoder.getVelocity() * -1);
        } else {
            SmartDashboard.putNumber("Bottom Shooter Velocity", shooterBottomMotorEncoder.getVelocity());
        }

        /**if (this.getCurrentCommand() != null) {
            SmartDashboard.putString("Intake Command", this.getCurrentCommand().getName());
        } else {
            SmartDashboard.putString("Intake Command", "none");
        }*/

        if (setShooterVelocity) {
            topPID.setReference(TopTargetVelocity, ControlType.kVelocity);
            bottomPID.setReference(BottomTargetVelocity, ControlType.kVelocity);
        }

    }

    private double getBottomVelocity() {
        return shooterBottomMotorEncoder.getVelocity();
    }

    private double getBottomRollerSpeed() {
        return (shooterBottomMotorEncoder.getVelocity() / 1.5);
    }

    private double getTopRollerSpeed() {
        return (shooterTopMotorEncoder.getVelocity() / 1.5);
    }

    public boolean isAtTargetVelocity() {
        return (Math.abs(shooterBottomMotorEncoder.getVelocity()) < BottomTargetVelocity + 75) && (Math.abs(shooterBottomMotorEncoder.getVelocity()) >  BottomTargetVelocity - 75) &&
               (Math.abs(shooterTopMotorEncoder.getVelocity()) < TopTargetVelocity + 75 && (Math.abs(shooterTopMotorEncoder.getVelocity()) > TopTargetVelocity - 75));
    }

    public boolean isAtTargetSpeed() {
        return (Math.abs(shooterBottomMotor.get()) < BottomTargetSpeed + 0.25) && (Math.abs(shooterBottomMotor.get()) >  BottomTargetSpeed - 0.25) &&
               (Math.abs(shooterTopMotor.get()) < TopTargetSpeed + 0.25 && (Math.abs(shooterTopMotor.get()) > TopTargetSpeed - 0.25));
    }

    public boolean isnot_TOOfastTooReverse() {
        return (Math.abs(shooterBottomMotorEncoder.getVelocity()) < 2000) && (Math.abs(shooterTopMotorEncoder.getVelocity()) < 2000);
    }

    private String getTopIdleMode() {
        return shooterTopMotor.getIdleMode().toString();
    }

    private String getBottomIdleMode() {
        return shooterBottomMotor.getIdleMode().toString();
    }

    private double getTopVelocity() {
        return shooterTopMotorEncoder.getVelocity();
    }

    public String getcurrentCommandS() {
        if (this.getCurrentCommand() != null) {
          return this.getCurrentCommand().getName();
        } else {
          return "none";
        }
      }

    public double getTopTargetVelocity() {
        return TopTargetVelocity;
    }

    public double getBottomTargetVelocity() {
        return BottomTargetVelocity;
    }

    public Command setShooterShootCommand() {
        return run(() -> setShooterShoot());
    }

    public Command AutoShooterCommand() {
        return this.runEnd(
            ()-> {
                setShooterShoot();
            },
            () -> {
                stopShooter();
        });
    }
    
}
