// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants{
  // General
  public static final class GeneralConstants{
    public static final double shooterAndPivotSetupTime = 3;
    public static final double autoShootEndTime = 5;
  }

  // Limit Switch and Sensor DIO Ports
  public static final class IOSwitchPorts{
    public static final int intakeSwitchID = 0;
    public static final int pivotUpLimitID = 1;
    public static final int pivotDownLimitID = 2; 
    public static final int leftClimberSwitch = 8;
    public static final int rightClimberSwitch = 9;
  }

  // Pathplanner
  public static final class PathPlannerConstants{
    public static final PIDConstants TranslationPID = new PIDConstants (10.5, 0.0, 0);
    public static final PIDConstants RotationPID = new PIDConstants(2, 0.0, 0.0);
  }

  // Shooter
  public static final class ShooterConstants{
    // Velocity For Shooting
    public static final double shooterShootVelocity = 3000;
    public static final double shooterAmpVelocity = 1000;
    public static final double shooterInVelocity = -500;
    // Shooter Motors
    public static final int topShooterMotorID = 30;
    public static final int bottomShooterMotorID = 31;
  }

  //Intake
  public static final class IntakeConstants{
    // Intake Speeds
    public static final double intakeInSpeed = .35;
    public static final double intakeFeedSpeed = 0.55;
    public static final double intakeReverseSpeed = -.35;
    // Intake Motors
    public static final int leftIntakeMotorID = 32;
    public static final int rightIntakeMotorID = 33;
  }

  //Pivot
  public static final class PivotConstants{
    //All Angles Measred from floor to top plate of shooter
    // Pivot Shooting Angles
    public static final double subwooferShootAngle = 60;
    public static final double podiumShootAngle = 55;
    public static final double ampShootAngle = 100;
    // Misc.
    public static final double pivotHomingSpeed = .2;
    public static final double startRangelowerAngle = 60;
    public static final double intakeMaxAngle = 50;
    public static final double intakeIdealAngle = 40;
    // Positions Of Limit Switches
    public static final double pivotBottomSwitchAngle = 31;
    public static final double pivotTopSwitchAngle = 124;
    //Minimum and Maximum Pivot Angles
    public static final double pivotMINAngle = 32;
    public static final double pivotMAXangle = 123;
    public static final double pivotToleranceAngle = 2;
    // Pivot Encoder Setup
    //                                                                13:1 Gear Ratio
    public static final double pivotAbsoluteEncoderConversionFactor = .07692307692;
    //                                                                25:1 and 13:1 Gear Ratio
    public static final double pivotrelativeEncoderConversionFactor = .00307692307;
    // Pivot PID Values
    public static final double pivotPID_kP = 8;
    public static final double pivotPID_kI = 0;
    public static final double pivotPID_kD = 0;
    //Pivot Motors
    public static final int leftPivotMotortID = 20;
    public static final int rightPivotMotorID = 21;
  }

  //SwerveDrive
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton {
    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);
    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }


  public static final class OperatorConstants{
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  // Blinkin Colors
  public static class Colors {
    public double fP_Rainbow         = -0.99;
    public double fP_Rainbow_party   = -0.97;
    public double fP_Rainbow_Ocean   = -0.95;
    public double fP_Rainbow_lava    = -0.93;
    public double fP_Rainbow_Forest  = -0.91;
    public double fP_Rainbow_Glitter = -0.89;
    public double fP_Confetti        = -0.87;
    public double fP_Shot_Red        = -0.85;
    public double fp_shot_Blue       = -0.83;
    public double fP_Shot_White      = -0.81;
    public double fP_Sinelon_Rainbow = -0.79;
    public double fP_Sinelon_Party   = -0.77;
    public double fP_Sinelon_Ocean   = -0.75;
    public double fP_Sinelon_Lava    = -0.73;
    public double fP_Sinelon_Forest  = -0.71;
    public double fP_BPM_Rainbow     = -0.69;
    public double fP_BPM_Party       = -0.67;
    public double fP_BPM_Ocean       = -0.65;
    public double fP_BPM_Lava        = -0.63;
    public double fP_BPM_Forest      = -0.61;
    public double fP_Fire_Med        = -0.59;
    public double fP_Fire_Large      = -0.57;
    public double fP_Twinkle_Rainbow = -0.55;
    public double fP_Twinkle_Party   = -0.53;
    public double fP_Twinkle_Ocean   = -0.51;
    public double fP_Twinkle_Lava    = -0.49;
    public double fP_Twinkle_Forest  = -0.47;
    public double fP_colWav_Rainbow  = -0.45;
    public double fP_colWav_Party    = -0.43;
    public double fP_colWav_Ocean    = -0.41;
    public double fP_colWav_Lava     = -0.39;
    public double fP_colWav_Forest   = -0.37;
    public double fP_LarScan_Red     = -0.35;
    public double fP_LarScan_gray    = -0.33;
    public double fP_LC_Red          = -0.31;
    public double fP_LC_Blue         = -0.29;
    public double fP_LC_gray         = -0.27;
    public double fP_HeartBeat_Red   = -0.25;
    public double fP_HeartBeat_Blue  = -0.23;
    public double fP_HeartBeat_white = -0.21;
    public double fP_HeartBeat_Gray  = -0.19;
    public double fixPal_Breath_Red  = -0.17;
    public double fixPal_Breath_Blue = -0.15;
    public double fixPal_Breath_Gray = -0.13;
    public double fixPal_Stobe_Red   = -0.11;
    public double fixPal_Stobe_Blue  = -0.09;
    public double fixPal_Stobe_Gold  = -0.07;
    public double fixPal_Stobe_white = -0.05;
    public double c1E2E_blend2Blck   = -0.03;
    public double c1LarScan          = -0.01;
    public double c1LightChase        = 0.01;
    public double c1HeartBeat_Slow    = 0.03;
    public double c1HeartBeat_Med     = 0.05;
    public double c1HeartBeat_Fast    = 0.07;
    public double c1BreathSlow        = 0.09;
    public double c1BreathFast        = 0.11;
    public double c1Shot              = 0.13;
    public double c1Strobe            = 0.15;
    public double c2_E2E_blend2Blck   = 0.17;
    public double c2LarScan           = 0.21;
    public double c2LightChase        = 0.21;
    public double c2Hearbeat_Slow     = 0.23;
    public double c2HearBeat_Med      = 0.25;
    public double c2HearBeat_Fast     = 0.27;
    public double c2BreathSlow        = 0.29;
    public double c2BreathFast        = 0.31;
    public double pat2_shot           = 0.33;
    public double strobeC2            = 0.35;
    public double sparkleC1C2         = 0.37;
    public double sparkleC2C1         = 0.39;
    public double colorGradiant       = 0.41;
    public double BPM_C1C2            = 0.43;
    public double e2eBlendC1C2        = 0.45;
    public double endToEndBlend       = 0.47;
    public double noBlendColors       = 0.49;
    public double twinkles            = 0.51;
    public double colorWaves          = 0.53;
    public double sinelon             = 0.55;
    public double hotPink             = 0.57;
    public double darkRed             = 0.59;
    public double red                 = 0.61;
    public double redOrange           = 0.63;
    public double orange              = 0.65;
    public double gold                = 0.67;
    public double yellow              = 0.69;
    public double lawnGreen           = 0.71;
    public double lime                = 0.73;
    public double darkGreen           = 0.75;
    public double green               = 0.77;
    public double blue_green          = 0.79;
    public double aqua                = 0.81;
    public double skyBlue             = 0.83;
    public double dark_blue           = 0.85;
    public double blue                = 0.87;
    public double blueViolet          = 0.89;
    public double purple              = 0.91;
    public double white               = 0.93;
    public double gray                = 0.95;
    public double dark_gray           = 0.97;
    public double black               = 0.99;
  }

}