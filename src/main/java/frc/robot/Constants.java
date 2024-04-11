// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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

  public static class VisionConstants {
    /** Physical location of the apriltag camera on the robot, relative to the center of the robot. */
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = new Transform3d(
      new Translation3d(-0.06, 0.2, -0.2127),
      new Rotation3d(0.0, degreesToRadians(15.0), degreesToRadians(-3.0))
    );
    
    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite alliance
    public static final Pose2d FLIPPING_POSE = new Pose2d(
        new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
        new Rotation2d(Math.PI));

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
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
    public static final double intakeFeedSpeed = .35;
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
    public double pat1_larscan       = -0.01;
    public double pat2_larScan       = 0.19;
    public double fix_rain           = -0.99;
    public double fix_rainParty      = -0.97;
    public double fix_ocean          = -0.95;
    public double fix_Lave           = -0.93;
    public double fix_forest         = -0.91;
    public double shot_Blue          = -0.83;
    public double pat2_lightChase     = 0.21;
    public double pat2_shot           = 0.33;
    public double endToEndBlend       = 0.47;
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