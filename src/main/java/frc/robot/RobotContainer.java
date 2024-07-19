// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Colors;
import frc.robot.commands.Intake.FeedAuto;
import frc.robot.commands.Intake.IntakeFeedCommand;
import frc.robot.commands.Intake.IntakeFeedV2Command;
import frc.robot.commands.Intake.IntakeInCommand;
import frc.robot.commands.Intake.IntakeInCommandBeamBrake;
import frc.robot.commands.Intake.IntakeOutCommand;
import frc.robot.commands.Pivot.PivotTriggerCommand;
import frc.robot.commands.Pivot.stopPivotCommand;
import frc.robot.commands.Shooter.AutoShootCommand;
import frc.robot.commands.Shooter.ShooterAmpCommand;
import frc.robot.commands.Shooter.ShooterAmpCommandV2;
import frc.robot.commands.Shooter.ShooterInCommand;
import frc.robot.commands.Shooter.ShooterSHUTTLECommand;
import frc.robot.commands.Shooter.ShooterShootCommand;
import frc.robot.commands.Shooter.ShooterShootV2Command;
import frc.robot.commands.Shooter.ShooterFun.ShooterFunCommand;
import frc.robot.commands.Shooter.ShooterFun.ShooterFunLongCommand;
import frc.robot.commands.Shooter.ShooterFun.ShooterFunLongV2Command;
import frc.robot.commands.Shooter.ShooterFun.ShooterFunLongV3Command;
import frc.robot.commands.Shooter.ShooterFun.ShooterFunMIDCommand;
import frc.robot.commands.Shooter.ShooterFun.ShooterFunV2Command;
import frc.robot.commands.Shooter.ShooterFun.ShooterFunV3Command;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.TelopDrive;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ServoTest;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  @SuppressWarnings({ "unused" })
  public static final CommandJoystick leftJoystick = new CommandJoystick(0);
  @SuppressWarnings({ "unused" })
  public static final CommandJoystick rightJoystick = new CommandJoystick(1);
  @SuppressWarnings({ "unused" })
  public static final XboxController xboxController = new XboxController(2);
  @SuppressWarnings({ "unused" })
  public final CommandXboxController xboxControllerCommand = new CommandXboxController(2);
  public final PS5Controller ps5Controller = new PS5Controller(3);
  public final CommandPS5Controller commandPS5controller = new CommandPS5Controller(3);

  // Subsystems
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem(xboxControllerCommand);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final Blinkin blinkin = new Blinkin();
  private final ServoTest servoTest = new ServoTest();

  // Intake
  private final IntakeInCommand intakeInCommand = new IntakeInCommand(intakeSubsystem);
  private final IntakeOutCommand intakeOutCommand = new IntakeOutCommand(intakeSubsystem, shooterSubsystem);
  private final IntakeFeedCommand intakeFeedCommand = new IntakeFeedCommand(intakeSubsystem, shooterSubsystem);
  private final FeedAuto feedAuto = new FeedAuto(intakeSubsystem);
  private final IntakeFeedV2Command intakeFeedV2Command = new IntakeFeedV2Command(intakeSubsystem);
  private final IntakeInCommandBeamBrake intakeInCommandBeamBrake = new IntakeInCommandBeamBrake(intakeSubsystem);

  // Shooter
  private final ShooterShootCommand shooterShootCommand = new ShooterShootCommand(shooterSubsystem, pivotSubsystem::isTooLow);
  private final ShooterSHUTTLECommand shooterSHUTTLECommand = new ShooterSHUTTLECommand(shooterSubsystem, pivotSubsystem::isTooLow);
  private final ShooterInCommand shooterInCommand = new ShooterInCommand(shooterSubsystem);
  private final ShooterAmpCommand shooterAmpCommand = new ShooterAmpCommand(shooterSubsystem);
  private final AutoShootCommand autoShootCommand = new AutoShootCommand(intakeSubsystem, shooterSubsystem);
  private final ShooterShootV2Command shooterShootV2Command = new ShooterShootV2Command(shooterSubsystem, pivotSubsystem::isTooLow);
  private final ShooterAmpCommandV2 shooterAmpCommandV2 = new ShooterAmpCommandV2(shooterSubsystem);
  private final ShooterFunCommand shooterFunCommand = new ShooterFunCommand(shooterSubsystem);
  private final ShooterFunV2Command shooterFunV2Command = new ShooterFunV2Command(shooterSubsystem);
  private final ShooterFunV3Command shooterFunV3Command = new ShooterFunV3Command(shooterSubsystem);
  private final ShooterFunMIDCommand shooterFunMIDCommand = new ShooterFunMIDCommand(shooterSubsystem);
  private final ShooterFunLongCommand shooterFunLongCommand = new ShooterFunLongCommand(shooterSubsystem);
  private final ShooterFunLongV2Command shooterFunLongV2Command = new ShooterFunLongV2Command(shooterSubsystem);
  private final ShooterFunLongV3Command shooterFunLongV3Command = new ShooterFunLongV3Command(shooterSubsystem);

  // Pivot
  private final PivotTriggerCommand pivotTriggerCommand = new PivotTriggerCommand(pivotSubsystem);
  private final stopPivotCommand stopPivotCommand = new stopPivotCommand(pivotSubsystem);

  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  private final ShuffleboardTab robotTab = Shuffleboard.getTab("Robot");

  private final PowerDistribution powerDistribution = new PowerDistribution(9, ModuleType.kRev);

  //private final Notifier controlThread;

  /**private final CANSparkMax FLdriveMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax BLdriveMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax BRdriveMotor = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax FRdriveMotor = new CANSparkMax(5, MotorType.kBrushless);*/

  private final Colors colors = new Colors();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    //CameraServer.startAutomaticCapture().setResolution(640, 480);

    pivotSubsystem.setBrake();
    drivebase.setMotorBrake(true);
    shooterSubsystem.setShooterCoast();
    intakeSubsystem.setBrake();
    pivotSubsystem.setDefaultCommand(pivotTriggerCommand);
    
    NamedCommands.registerCommand("Shoot", shooterSubsystem.AutoShooterCommand().withTimeout(18));
    NamedCommands.registerCommand("Initial Feed", intakeSubsystem.setIntakeFeedCommand().withTimeout(.5));
    NamedCommands.registerCommand("Feed", intakeSubsystem.setIntakeFeedCommand().withTimeout(18));
        //NamedCommands.registerCommand("Feed44", intakeSubsystem.setIntakeFeedCommand().onlyWhile(null));
    NamedCommands.registerCommand("Intake", intakeInCommandBeamBrake);
    NamedCommands.registerCommand("Pivot Subwoofer", pivotSubsystem.setPivotShootSpeakerCommand().withTimeout(.75));
    NamedCommands.registerCommand("Pivot Intake", pivotSubsystem.setPivotIntakeCommand().withTimeout(1.5));
    NamedCommands.registerCommand("Pivot Far", pivotSubsystem.autoFront3Notes_Command().withTimeout(1.5));
    NamedCommands.registerCommand("Pivot Podium", pivotSubsystem.setPivotShootStageCommand());

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Nothing", new RunCommand(() -> {}));

    configure_FUN_Bindings();
    configureDashboard();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
      // Applies deadbands and inverts controls because joysticks
      // are back-right positive while robot
      // controls are front-left positive
      () -> MathUtil.applyDeadband(rightJoystick.getY(), 0.15),
      () -> MathUtil.applyDeadband(rightJoystick.getX(), 0.15),
      () -> -MathUtil.applyDeadband(leftJoystick.getX(), 0.15),
      () -> -MathUtil.applyDeadband(leftJoystick.getY(), 0.15)
    );

    AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
      () -> MathUtil.applyDeadband(rightJoystick.getY(), 0.15),
      () -> MathUtil.applyDeadband(rightJoystick.getX(), 0.15),
      () -> MathUtil.applyDeadband(leftJoystick.getX(), 0.15)
    );

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> HeadingCorrection() * HeadingCorrection() * MathUtil.applyDeadband(-rightJoystick.getY(), 0.155),
      () -> HeadingCorrection() * HeadingCorrection() * MathUtil.applyDeadband(-rightJoystick.getX(), 0.155),
      () -> MathUtil.applyDeadband(-leftJoystick.getX(), 0.155),
      () -> MathUtil.applyDeadband(-leftJoystick.getY(), 0.155)
    );

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(rightJoystick.getY(), 0.15),
      () -> MathUtil.applyDeadband(rightJoystick.getY(), 0.15),
      () -> MathUtil.applyDeadband(leftJoystick.getX(), 0.15)
    );

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(rightJoystick.getY(), 0.15),
        () -> MathUtil.applyDeadband(rightJoystick.getX(), 0.15),
        () -> MathUtil.applyDeadband(leftJoystick.getX(), 0.15)
    );

    TelopDrive closedFieldRel = new TelopDrive(
      drivebase,
      () -> HeadingCorrection() * MathUtil.applyDeadband(-rightJoystick.getY(), 0.075),
      () -> HeadingCorrection() * MathUtil.applyDeadband(-rightJoystick.getX(), 0.075),
      () -> MathUtil.applyDeadband(-leftJoystick.getX() * 0.75, 0.075), () -> true
    );

    /**TelopDrive closedFieldRel = new TelopDrive(
      drivebase,
      () -> HeadingCorrection() * MathUtil.applyDeadband(-ps5Controller.getLeftY(), 0.075),
      () -> HeadingCorrection() * MathUtil.applyDeadband(-ps5Controller.getLeftX(), 0.075),
      () -> MathUtil.applyDeadband(-ps5Controller.getRightX() * 0.75, 0.075), () -> true
    );*/

    //drivebase.setDefaultCommand(closedFieldRel);
  }

  private void configureDashboard() {
    final var pivotTab = Shuffleboard.getTab("Pivot");
    pivotSubsystem.addDashboardWidgets(pivotTab.getLayout("Pivot", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(5, 4));

    final var shooterAndIntakeTab = Shuffleboard.getTab("Shooter & Intake");
    shooterSubsystem.addDashboardWidgets(shooterAndIntakeTab.getLayout("Shooter", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(3, 3));
    intakeSubsystem.addDashboardWidgets(shooterAndIntakeTab.getLayout("Intake", BuiltInLayouts.kGrid).withPosition(5, 0).withSize(3, 3));

    final var servoTab = Shuffleboard.getTab("Servo");
    servoTest.addDashboardWidgets(servoTab.getLayout("Servo", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(4, 4));
    
    CameraServer.startAutomaticCapture(0).setResolution(640, 480);
    CameraServer.getVideo().getSource().setFPS(15);
    var camera = CameraServer.getVideo();

    driverTab.add("Auto", autoChooser).withPosition(0, 0).withSize(2, 1);
    //driverTab.add("Control Setup", controlChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 1).withSize(2, 1);
    driverTab.add(drivebase.getSwerveField()).withWidget(BuiltInWidgets.kField).withPosition(2, 0).withSize(6, 4);
    driverTab.add(camera.getSource()).withWidget(BuiltInWidgets.kCameraStream).withProperties(Map.of("showCrosshair", true, "showControls", true)).withPosition(5, 0).withSize(5, 4);
    //driverTab.add(CameraServer.startAutomaticCapture()).withWidget(BuiltInWidgets.kCameraStream).withProperties(Map.of("showCrosshair", true, "showControls", true)).withPosition(3, 0).withSize(6, 4);

    robotTab.add(powerDistribution).withWidget(BuiltInWidgets.kPowerDistribution).withPosition(2, 0).withSize(3, 4);

    //robotTab.addNumber("test" , this::getAvgMotorTemp).withWidget(BuiltInWidgets.kNumberBar).withPosition(0, 0).withSize(2, 1);

    //Shuffleboard.selectTab(ClimberTab.getTitle());
    //Shuffleboard.selectTab(shooterAndIntakeTab.getTitle());
    //Shuffleboard.selectTab(pivotTab.getTitle());
    Shuffleboard.selectTab(driverTab.getTitle());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  

  public void configure_FUN_Bindings() {

    //xboxControllerCommand.button(7).whileTrue(new InstantCommand(() -> servo.setAngle(20)));
    //xboxControllerCommand.button(8).whileTrue(new InstantCommand(() -> servo.setAngle(70)));

    //xboxControllerCommand.button(7).whileTrue(new InstantCommand(() -> servo.setPosition(1)));
    //xboxControllerCommand.button(8).whileTrue(new InstantCommand(() -> servo.setPosition(0.5)));

    //xboxControllerCommand.button(7).whileTrue(new InstantCommand(() -> servo.setSpeed(0.5)));
    //xboxControllerCommand.button(8).whileTrue(new InstantCommand(() -> servo.setSpeed(0.25)));

    //xboxControllerCommand.button(7).whileTrue(new InstantCommand(() -> servo.set(0.75)));
    //xboxControllerCommand.button(8).whileTrue(new InstantCommand(() -> servo.set(0.25)));

    xboxControllerCommand.rightBumper().whileTrue(
      Commands.startEnd(() -> servoTest.setServoPosition(0.0), servoTest::setServoDisabled, servoTest));

    xboxControllerCommand.leftBumper().whileTrue(
      Commands.startEnd(() -> servoTest.setServoPosition(1.0), servoTest::setServoDisabled, servoTest));
  }

  private void configure_Grace_Bindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    leftJoystick.button(7).onTrue(new InstantCommand(drivebase::zeroGyro));


    xboxControllerCommand.b().whileTrue(pivotSubsystem.setPivotShootSpeakerCommand());
    xboxControllerCommand.a().whileTrue(pivotSubsystem.setPivotIntakeCommand());
    xboxControllerCommand.y().whileTrue(pivotSubsystem.setPivot_Finish_AMPCommand());
    xboxControllerCommand.x().whileTrue(pivotSubsystem.setPivotShootStageCommand());

    new Trigger(intakeSubsystem::hasNoteRAW).and(xboxControllerCommand.rightTrigger())
    .whileTrue(new InstantCommand(() -> xboxController.setRumble(RumbleType.kBothRumble, 0.75)).alongWith(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Stobe_Red))))
    .onFalse(new InstantCommand(() -> xboxController.setRumble(RumbleType.kBothRumble, 0.0)).alongWith(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Breath_Blue))));

    new Trigger(pivotSubsystem::isAimAtTargetPosition)
    .whileTrue(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Stobe_white)).andThen(new WaitCommand(2)).andThen(new InstantCommand(()-> blinkin.setCustomColor(colors.c2BreathSlow))))
    .onFalse(new InstantCommand(()-> blinkin.setCustomColor(colors.fixPal_Breath_Blue)));
    
    xboxControllerCommand.rightBumper().whileTrue(intakeOutCommand);

    xboxControllerCommand.leftBumper().whileTrue(shooterAmpCommand);

    xboxControllerCommand.leftTrigger().whileTrue(shooterShootCommand);

    xboxControllerCommand.rightTrigger().whileTrue(intakeInCommand);
  }

  public void configure_NoXbox_Bindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    leftJoystick.button(7).onTrue(new InstantCommand(drivebase::zeroGyro));


    xboxControllerCommand.b().whileTrue(pivotSubsystem.setPivotShootSpeakerCommand());
    xboxControllerCommand.a().whileTrue(pivotSubsystem.setPivotIntakeCommand());
    xboxControllerCommand.y().whileTrue(pivotSubsystem.setPivot_Finish_AMPCommand());
    xboxControllerCommand.x().whileTrue(pivotSubsystem.setPivotShootStageCommand());

    new Trigger(intakeSubsystem::hasNoteRAW).and(xboxControllerCommand.rightTrigger())
    .whileTrue(new InstantCommand(() -> xboxController.setRumble(RumbleType.kBothRumble, 0.75)).alongWith(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Stobe_Red))))
    .onFalse(new InstantCommand(() -> xboxController.setRumble(RumbleType.kBothRumble, 0.0)).alongWith(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Breath_Blue))));

    new Trigger(pivotSubsystem::isAimAtTargetPosition)
    .whileTrue(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Stobe_white)).andThen(new WaitCommand(2)).andThen(new InstantCommand(()-> blinkin.setCustomColor(colors.c2BreathSlow))))
    .onFalse(new InstantCommand(()-> blinkin.setCustomColor(colors.fixPal_Breath_Blue)));
    
    xboxControllerCommand.rightBumper().whileTrue(intakeOutCommand);

    xboxControllerCommand.leftBumper().whileTrue(shooterAmpCommand);

    xboxControllerCommand.leftTrigger().whileTrue(shooterShootCommand);

    xboxControllerCommand.rightTrigger().whileTrue(intakeInCommand);
  }

  public void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    leftJoystick.button(7).onTrue(new InstantCommand(drivebase::zeroGyro));


    xboxControllerCommand.b().whileTrue(pivotSubsystem.setPivotShootSpeakerCommand());
    xboxControllerCommand.a().whileTrue(pivotSubsystem.setPivotIntakeCommand());
    xboxControllerCommand.y().whileTrue(pivotSubsystem.setPivot_Finish_AMPCommand());
    xboxControllerCommand.x().whileTrue(pivotSubsystem.setPivotShootStageCommand());

    new Trigger(intakeSubsystem::hasNoteRAW).and(xboxControllerCommand.rightTrigger())
    .whileTrue(new InstantCommand(() -> xboxController.setRumble(RumbleType.kBothRumble, 0.75)).alongWith(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Stobe_Red))))
    .onFalse(new InstantCommand(() -> xboxController.setRumble(RumbleType.kBothRumble, 0.0)).alongWith(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Breath_Blue))));

    new Trigger(pivotSubsystem::isAimAtTargetPosition)
    .whileTrue(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Stobe_white)).andThen(new WaitCommand(2)).andThen(new InstantCommand(()-> blinkin.setCustomColor(colors.c2BreathSlow))))
    .onFalse(new InstantCommand(()-> blinkin.setCustomColor(colors.fixPal_Breath_Blue)));
    
    xboxControllerCommand.rightBumper().whileTrue(intakeOutCommand);

    xboxControllerCommand.leftBumper().whileTrue(shooterAmpCommand);

    xboxControllerCommand.leftTrigger().whileTrue(shooterShootCommand);

    xboxControllerCommand.rightTrigger().whileTrue(intakeInCommand);
  }

  private void configure_Cameron_Bindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    leftJoystick.button(7).onTrue(new InstantCommand(drivebase::zeroGyro));

    new Trigger(intakeSubsystem::hasNoteRAW).and(xboxControllerCommand.rightTrigger())
    .whileTrue(new InstantCommand(() -> xboxController.setRumble(RumbleType.kBothRumble, 0.75)).alongWith(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Stobe_Red))))
    .onFalse(new InstantCommand(() -> xboxController.setRumble(RumbleType.kBothRumble, 0.0)).alongWith(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Breath_Blue))));

    new Trigger(pivotSubsystem::isAimAtTargetPosition)
    .whileTrue(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Stobe_white)).andThen(new WaitCommand(1)).andThen(new InstantCommand(()-> blinkin.setCustomColor(colors.c2BreathSlow))))
    .onFalse(new InstantCommand(()-> blinkin.setCustomColor(colors.fixPal_Breath_Blue)));

    new Trigger(xboxControllerCommand.rightTrigger().and(xboxControllerCommand.leftTrigger().negate()).and(xboxControllerCommand.leftBumper().negate()))
    .whileTrue(intakeInCommand.alongWith(pivotSubsystem.setPivotIntakeCommand()).until(intakeSubsystem::hasNoteRAW));

    new Trigger(xboxControllerCommand.leftBumper().and(xboxControllerCommand.button(8).negate()).and(shooterSubsystem::isAtTargetSpeed).and(pivotSubsystem::isAimAtTargetPosition).and(xboxControllerCommand.rightTrigger()))
    .whileTrue(intakeFeedV2Command);

    new Trigger(xboxControllerCommand.leftTrigger().and(xboxControllerCommand.y().negate()).and(xboxControllerCommand.a().negate()))
    .whileTrue(shooterShootCommand.alongWith(pivotSubsystem.setPivotShootSpeakerCommand()));

    new Trigger(xboxControllerCommand.leftTrigger().and(xboxControllerCommand.y()))
    .whileTrue(shooterShootV2Command.alongWith(pivotSubsystem.setPivotShootStageCommand()));

    new Trigger(xboxControllerCommand.leftTrigger().and(xboxControllerCommand.a()))
    .whileTrue(shooterSHUTTLECommand.alongWith(pivotSubsystem.setPivotShootStageCommand()));

    new Trigger(xboxControllerCommand.leftTrigger().and(shooterSubsystem::isAtTargetVelocity).and(xboxControllerCommand.rightTrigger()).and(xboxControllerCommand.button(8).negate()))
    .whileTrue(intakeFeedV2Command);
    
    new Trigger(xboxControllerCommand.leftBumper().and(xboxControllerCommand.button(8).negate()))
    .whileTrue(shooterAmpCommand.alongWith(pivotSubsystem.setPivot_Finish_AMPCommand()));

    new Trigger(xboxControllerCommand.rightBumper().and(shooterSubsystem::isnot_TOOfastTooReverse))
    .whileTrue(intakeOutCommand);

    new Trigger(xboxControllerCommand.rightTrigger().and(xboxControllerCommand.button(8)))
    .whileTrue(intakeFeedV2Command);

    new Trigger(xboxControllerCommand.leftBumper().and(xboxControllerCommand.button(8)))
    .whileTrue(shooterAmpCommandV2);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }

  public void setDriveMode() {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  private int HeadingCorrection() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        return -1;
      }
     else {
        return 1;
    }
  }
    return -1;
  
  }

}

