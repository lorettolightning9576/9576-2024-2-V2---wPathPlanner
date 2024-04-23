// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Colors;
import frc.robot.commands.Intake.FeedAuto;
import frc.robot.commands.Intake.IntakeFeedCommand;
import frc.robot.commands.Intake.IntakeFeedV2Command;
import frc.robot.commands.Intake.IntakeInCommand;
import frc.robot.commands.Intake.IntakeOutCommand;
import frc.robot.commands.Pivot.PivotTriggerCommand;
import frc.robot.commands.Pivot.stopPivotCommand;
import frc.robot.commands.Shooter.AutoShootCommand;
import frc.robot.commands.Shooter.ShooterAmpCommand;
import frc.robot.commands.Shooter.ShooterInCommand;
import frc.robot.commands.Shooter.ShooterShootCommand;
import frc.robot.commands.Shooter.ShooterShootV2Command;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.TelopDrive;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
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

  // Subsystems
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem(xboxControllerCommand);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final Blinkin blinkin = new Blinkin();

  // Intake
  private final IntakeInCommand intakeInCommand = new IntakeInCommand(intakeSubsystem, xboxControllerCommand);
  private final IntakeOutCommand intakeOutCommand = new IntakeOutCommand(intakeSubsystem, shooterSubsystem);
  private final IntakeFeedCommand intakeFeedCommand = new IntakeFeedCommand(intakeSubsystem, shooterSubsystem);
  private final FeedAuto feedAuto = new FeedAuto(intakeSubsystem);
  private final IntakeFeedV2Command intakeFeedV2Command = new IntakeFeedV2Command(intakeSubsystem);

  // Shooter
  private final ShooterShootCommand shooterShootCommand = new ShooterShootCommand(shooterSubsystem, pivotSubsystem::isTooLow);
  private final ShooterInCommand shooterInCommand = new ShooterInCommand(shooterSubsystem);
  private final ShooterAmpCommand shooterAmpCommand = new ShooterAmpCommand(shooterSubsystem);
  private final AutoShootCommand autoShootCommand = new AutoShootCommand(intakeSubsystem, shooterSubsystem);
  private final ShooterShootV2Command shooterShootV2Command = new ShooterShootV2Command(shooterSubsystem, pivotSubsystem::isTooLow);

  // Pivot
  private final PivotTriggerCommand pivotTriggerCommand = new PivotTriggerCommand(pivotSubsystem);
  private final stopPivotCommand stopPivotCommand = new stopPivotCommand(pivotSubsystem);

  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  private final ShuffleboardTab robotTab = Shuffleboard.getTab("Robot");

  private final PowerDistribution powerDistribution = new PowerDistribution(9, ModuleType.kRev);

  private final Colors colors = new Colors();

  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Command> controlChooser = new SendableChooser<>();

  public RobotContainer() {
    CameraServer.startAutomaticCapture(0);

    pivotSubsystem.setBrake();
    drivebase.setMotorBrake(true);
    shooterSubsystem.setShooterCoast();
    intakeSubsystem.setBrake();

    pivotSubsystem.setDefaultCommand(pivotTriggerCommand);
    
    NamedCommands.registerCommand("Shoot", shooterSubsystem.AutoShooterCommand().withTimeout(18));
    NamedCommands.registerCommand("Initial Feed", intakeSubsystem.setIntakeFeedCommand().withTimeout(.5));
    NamedCommands.registerCommand("Feed", intakeSubsystem.setIntakeFeedCommand().withTimeout(1.5));
    NamedCommands.registerCommand("Intake", intakeInCommand);
    NamedCommands.registerCommand("Pivot Subwoofer", pivotSubsystem.setPivotShootSpeakerCommand().withTimeout(1));
    NamedCommands.registerCommand("Pivot Intake", pivotSubsystem.setPivotIntakeCommand().withTimeout(1.5));
    NamedCommands.registerCommand("Pivot Far", pivotSubsystem.autoFront3Notes_Command().withTimeout(1.5));
    NamedCommands.registerCommand("Pivot Podium", pivotSubsystem.setPivotShootStageCommand());

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Nothing", new RunCommand(() -> {}));

    //controlChooser.setDefaultOption("Default", new InstantCommand(() -> configureBindings()));
    controlChooser.setDefaultOption("Default-CK", new InstantCommand(() -> configure_Cameron_Bindings()));
    controlChooser.addOption("Default", new InstantCommand(() -> configureBindings()));
    controlChooser.addOption("Cameron", new InstantCommand(() -> configure_Cameron_Bindings()));
    controlChooser.addOption("Grace", new InstantCommand(() -> configure_Grace_Bindings()));
    controlChooser.addOption("No Xbox", new InstantCommand(() -> configure_NoXbox_Bindings()));


    if (controlChooser.getSelected().equals("Cameron")) {
      configure_Cameron_Bindings();
    } else if (controlChooser.getSelected().equals("Default-CK")) {
      configure_Cameron_Bindings();
    } else {
      configure_Cameron_Bindings();
    }
    //getControlSetupCommand();
    //configureBindings();
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
      () -> MathUtil.applyDeadband(-leftJoystick.getX() * 0.8, 0.075), () -> true
    );

    drivebase.setDefaultCommand(closedFieldRel);
  }

  private void configureDashboard() {
    final var pivotTab = Shuffleboard.getTab("Pivot");
    pivotSubsystem.addDashboardWidgets(pivotTab.getLayout("Pivot", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(5, 4));

    final var shooterAndIntakeTab = Shuffleboard.getTab("Shooter & Intake");
    shooterSubsystem.addDashboardWidgets(shooterAndIntakeTab.getLayout("Shooter", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(3, 3));
    intakeSubsystem.addDashboardWidgets(shooterAndIntakeTab.getLayout("Intake", BuiltInLayouts.kGrid).withPosition(5, 0).withSize(3, 3));
    final var climberTab = Shuffleboard.getTab("Climber");
    climberSubsystem.addDashboardWidgets(climberTab.getLayout("Climber", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(4, 3));



    driverTab.add("Auto", autoChooser).withPosition(0, 0).withSize(1, 1);
    driverTab.add("Control Setup", controlChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(1, 1).withSize(2, 1);
    //driverTab.add(new UsbCamera("camera", 0)).withWidget(BuiltInWidgets.kCameraStream).withProperties(Map.of("showCrosshair", true, "showControls", true)).withPosition(3, 0).withSize(5, 4);
    driverTab.add(CameraServer.putVideo("Intake Camera", 640, 480)).withWidget(BuiltInWidgets.kCameraStream).withProperties(Map.of("showCrosshair", true, "showControls", true)).withPosition(3, 0).withSize(5, 4);

    robotTab.add(powerDistribution).withWidget(BuiltInWidgets.kPowerDistribution).withPosition(1, 0).withSize(3, 4);

    /**driverTab.add(new HttpCamera("Intake Camera", "http://10.95.76.2"))
      .withWidget(BuiltInWidgets.kCameraStream)
      .withProperties(Map.of("showCrosshair", true, "showControls", true))
      .withSize(4, 5).withPosition(3, 0);*/
    //driverTab.addCamera("Intake Camera V2", null, CameraServer.startAutomaticCapture().getPath());

    /**driverTab.add(new HttpCamera("Intake USB Camera", CameraServer.startAutomaticCapture().getPath()))
      .withWidget(BuiltInWidgets.kCameraStream)
      .withProperties(Map.of("showCrosshair", true, "showControls", true))
      .withSize(4, 5).withPosition(3, 0);*/

    //driverTab.add(CameraServer.getVideo());

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
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    leftJoystick.button(7).onTrue(new InstantCommand(drivebase::zeroGyro));
    //rightJoystick.button(7).onTrue(new InstantCommand(() -> intakeSubsystem.setBrake()));
    //rightJoystick.button(8).onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeCoast()));
    //rightJoystick.button(9).onTrue(new InstantCommand(() -> pivotSubsystem.setBrake()));
    //rightJoystick.button(10).onTrue(new InstantCommand(() -> pivotSubsystem.setCoast()));

    leftJoystick.povUp().whileTrue(climberSubsystem.raiseLeftArmCommand());
    rightJoystick.povUp().whileTrue(climberSubsystem.raiseRightArmCommand());
    leftJoystick.povDown().whileTrue(climberSubsystem.lower_LEFT_ArmCommand());
    rightJoystick.povDown().whileTrue(climberSubsystem.lower_RIGHT_ArmCommand());

    xboxControllerCommand.povUp().whileTrue(climberSubsystem.raise_BOTH_ArmCommand());
    xboxControllerCommand.povDown()
    .whileTrue(new InstantCommand(() -> climberSubsystem.fastLower = true).alongWith(climberSubsystem.lower_BOTH_ArmCommand()))
    .onFalse(new InstantCommand(() -> climberSubsystem.fastLower = false));
    
    rightJoystick.button(1)
    .whileTrue(new InstantCommand(() -> climberSubsystem.fastLower = true).alongWith(new InstantCommand(() -> climberSubsystem.slowRaise = true)))
    .onFalse(new InstantCommand(() -> climberSubsystem.fastLower = false).alongWith(new InstantCommand(() -> climberSubsystem.slowRaise = false)));


    leftJoystick.povDown().and(rightJoystick.povDown()).whileTrue(climberSubsystem.lower_BOTH_ArmCommand());
    leftJoystick.povUp().and(rightJoystick.povUp()).whileTrue(climberSubsystem.raise_BOTH_ArmCommand());


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
    //rightJoystick.button(7).onTrue(new InstantCommand(() -> intakeSubsystem.setBrake()));
    //rightJoystick.button(8).onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeCoast()));
    //rightJoystick.button(9).onTrue(new InstantCommand(() -> pivotSubsystem.setBrake()));
    //rightJoystick.button(10).onTrue(new InstantCommand(() -> pivotSubsystem.setCoast()));

    leftJoystick.povUp().whileTrue(climberSubsystem.raiseLeftArmCommand());
    rightJoystick.povUp().whileTrue(climberSubsystem.raiseRightArmCommand());
    leftJoystick.povDown().whileTrue(climberSubsystem.lower_LEFT_ArmCommand());
    rightJoystick.povDown().whileTrue(climberSubsystem.lower_RIGHT_ArmCommand());

    xboxControllerCommand.povUp().whileTrue(climberSubsystem.raise_BOTH_ArmCommand());
    xboxControllerCommand.povDown()
    .whileTrue(new InstantCommand(() -> climberSubsystem.fastLower = true).alongWith(climberSubsystem.lower_BOTH_ArmCommand()))
    .onFalse(new InstantCommand(() -> climberSubsystem.fastLower = false));
    
    rightJoystick.button(1)
    .whileTrue(new InstantCommand(() -> climberSubsystem.fastLower = true).alongWith(new InstantCommand(() -> climberSubsystem.slowRaise = true)))
    .onFalse(new InstantCommand(() -> climberSubsystem.fastLower = false).alongWith(new InstantCommand(() -> climberSubsystem.slowRaise = false)));


    leftJoystick.povDown().and(rightJoystick.povDown()).whileTrue(climberSubsystem.lower_BOTH_ArmCommand());
    leftJoystick.povUp().and(rightJoystick.povUp()).whileTrue(climberSubsystem.raise_BOTH_ArmCommand());


    xboxControllerCommand.b().whileTrue(pivotSubsystem.setPivotShootSpeakerCommand());
    xboxControllerCommand.a().whileTrue(pivotSubsystem.setPivotIntakeCommand());
    //xboxControllerCommand.y().whileTrue(pivotSubsystem.setPivot_Finish_AMPCommand());
    xboxControllerCommand.x().whileTrue(pivotSubsystem.setPivotShootStageCommand());

    new Trigger(intakeSubsystem::hasNoteRAW).and(xboxControllerCommand.rightTrigger())
    .whileTrue(new InstantCommand(() -> xboxController.setRumble(RumbleType.kBothRumble, 0.75)).alongWith(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Stobe_Red))))
    .onFalse(new InstantCommand(() -> xboxController.setRumble(RumbleType.kBothRumble, 0.0)).alongWith(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Breath_Blue))));

    new Trigger(pivotSubsystem::isAimAtTargetPosition)
    .whileTrue(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Stobe_white)).andThen(new WaitCommand(1)).andThen(new InstantCommand(()-> blinkin.setCustomColor(colors.c2BreathSlow))))
    .onFalse(new InstantCommand(()-> blinkin.setCustomColor(colors.fixPal_Breath_Blue)));

    /**new Trigger(xboxControllerCommand.rightTrigger()).and(xboxControllerCommand.leftTrigger().negate()).and(xboxControllerCommand.leftBumper().negate())
    .whileTrue(intakeInCommand.alongWith(pivotSubsystem.setPivotIntakeCommand()).until(intakeSubsystem::hasNoteRAW));*/

    new Trigger(xboxControllerCommand.rightTrigger().and(xboxControllerCommand.leftTrigger().negate()).and(xboxControllerCommand.leftBumper().negate()))
    .whileTrue(intakeInCommand.alongWith(pivotSubsystem.setPivotIntakeCommand()).until(intakeSubsystem::hasNoteRAW));

    new Trigger(xboxControllerCommand.leftBumper().and(shooterSubsystem::isAtTargetSpeed).and(xboxControllerCommand.rightTrigger()))
    .whileTrue(intakeFeedV2Command);

    new Trigger(xboxControllerCommand.leftTrigger().and(xboxControllerCommand.y().negate()))
    .whileTrue(shooterShootCommand.alongWith(pivotSubsystem.setPivotShootSpeakerCommand()));

    new Trigger(xboxControllerCommand.leftTrigger().and(xboxControllerCommand.y()))
    .whileTrue(shooterShootV2Command.alongWith(pivotSubsystem.setPivotShootStageCommand()));

    /**new Trigger(xboxControllerCommand.leftTrigger().and(xboxControllerCommand.y()))
    .whileTrue(shooterShootCommand.alongWith(pivotSubsystem.setPivotShootStageCommand()));*/

    new Trigger(xboxControllerCommand.leftTrigger()).and(shooterSubsystem::isAtTargetVelocity).and(xboxControllerCommand.rightTrigger())
    .whileTrue(intakeFeedV2Command);
    
    xboxControllerCommand.leftBumper().whileTrue(shooterAmpCommand.alongWith(pivotSubsystem.setPivot_Finish_AMPCommand()));

    xboxControllerCommand.rightBumper().whileTrue(intakeOutCommand);

    //xboxControllerCommand.leftBumper().whileTrue(shooterAmpCommand);

    //xboxControllerCommand.leftTrigger().whileTrue(shooterShootCommand);

    //xboxControllerCommand.rightTrigger().whileTrue(intakeInCommand);
  }

  private void configure_Grace_Bindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    leftJoystick.button(7).onTrue(new InstantCommand(drivebase::zeroGyro));
    //rightJoystick.button(7).onTrue(new InstantCommand(() -> intakeSubsystem.setBrake()));
    //rightJoystick.button(8).onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeCoast()));
    //rightJoystick.button(9).onTrue(new InstantCommand(() -> pivotSubsystem.setBrake()));
    //rightJoystick.button(10).onTrue(new InstantCommand(() -> pivotSubsystem.setCoast()));

    leftJoystick.povUp().whileTrue(climberSubsystem.raiseLeftArmCommand());
    rightJoystick.povUp().whileTrue(climberSubsystem.raiseRightArmCommand());
    leftJoystick.povDown().whileTrue(climberSubsystem.lower_LEFT_ArmCommand());
    rightJoystick.povDown().whileTrue(climberSubsystem.lower_RIGHT_ArmCommand());

    xboxControllerCommand.povUp().whileTrue(climberSubsystem.raise_BOTH_ArmCommand());
    xboxControllerCommand.povDown()
    .whileTrue(new InstantCommand(() -> climberSubsystem.fastLower = true).alongWith(climberSubsystem.lower_BOTH_ArmCommand()))
    .onFalse(new InstantCommand(() -> climberSubsystem.fastLower = false));
    
    rightJoystick.button(1)
    .whileTrue(new InstantCommand(() -> climberSubsystem.fastLower = true).alongWith(new InstantCommand(() -> climberSubsystem.slowRaise = true)))
    .onFalse(new InstantCommand(() -> climberSubsystem.fastLower = false).alongWith(new InstantCommand(() -> climberSubsystem.slowRaise = false)));


    leftJoystick.povDown().and(rightJoystick.povDown()).whileTrue(climberSubsystem.lower_BOTH_ArmCommand());
    leftJoystick.povUp().and(rightJoystick.povUp()).whileTrue(climberSubsystem.raise_BOTH_ArmCommand());


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

  private void configure_NoXbox_Bindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    leftJoystick.button(7).onTrue(new InstantCommand(drivebase::zeroGyro));
    //rightJoystick.button(7).onTrue(new InstantCommand(() -> intakeSubsystem.setBrake()));
    //rightJoystick.button(8).onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeCoast()));
    //rightJoystick.button(9).onTrue(new InstantCommand(() -> pivotSubsystem.setBrake()));
    //rightJoystick.button(10).onTrue(new InstantCommand(() -> pivotSubsystem.setCoast()));

    leftJoystick.povUp().whileTrue(climberSubsystem.raiseLeftArmCommand());
    rightJoystick.povUp().whileTrue(climberSubsystem.raiseRightArmCommand());
    leftJoystick.povDown().whileTrue(climberSubsystem.lower_LEFT_ArmCommand());
    rightJoystick.povDown().whileTrue(climberSubsystem.lower_RIGHT_ArmCommand());

    xboxControllerCommand.povUp().whileTrue(climberSubsystem.raise_BOTH_ArmCommand());
    xboxControllerCommand.povDown()
    .whileTrue(new InstantCommand(() -> climberSubsystem.fastLower = true).alongWith(climberSubsystem.lower_BOTH_ArmCommand()))
    .onFalse(new InstantCommand(() -> climberSubsystem.fastLower = false));
    
    rightJoystick.button(1)
    .whileTrue(new InstantCommand(() -> climberSubsystem.fastLower = true).alongWith(new InstantCommand(() -> climberSubsystem.slowRaise = true)))
    .onFalse(new InstantCommand(() -> climberSubsystem.fastLower = false).alongWith(new InstantCommand(() -> climberSubsystem.slowRaise = false)));


    leftJoystick.povDown().and(rightJoystick.povDown()).whileTrue(climberSubsystem.lower_BOTH_ArmCommand());
    leftJoystick.povUp().and(rightJoystick.povUp()).whileTrue(climberSubsystem.raise_BOTH_ArmCommand());


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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }

  public Command getControlSetupCommand() {
    return controlChooser.getSelected();
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

