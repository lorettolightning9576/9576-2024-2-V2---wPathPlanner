// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Colors;
import frc.robot.commands.Intake.FeedAuto;
import frc.robot.commands.Intake.IntakeFeedCommand;
import frc.robot.commands.Intake.IntakeInCommand;
import frc.robot.commands.Intake.IntakeOutCommand;
import frc.robot.commands.Pivot.PivotTriggerCommand;
import frc.robot.commands.Pivot.stopPivotCommand;
import frc.robot.commands.Shooter.AutoShootCommand;
import frc.robot.commands.Shooter.ShooterAmpCommand;
import frc.robot.commands.Shooter.ShooterInCommand;
import frc.robot.commands.Shooter.ShooterShootCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.PhotonAlignCommand;
import frc.robot.commands.swervedrive.drivebase.TelopDrive;
import frc.robot.commands.swervedrive.drivebase.TelopDrive_wVisionAlign;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  private final PhotonCamera photonCamera = new PhotonCamera("camera");

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
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(photonCamera, drivebase);

  //private final PhotonAlignCommand photonAlignCommand = new PhotonAlignCommand(photonCamera, drivebase, () -> visionSubsystem.getCurrentPose());

  // Intake
  private final IntakeInCommand intakeInCommand = new IntakeInCommand(intakeSubsystem, xboxControllerCommand);
  private final IntakeOutCommand intakeOutCommand = new IntakeOutCommand(intakeSubsystem, shooterSubsystem);
  private final IntakeFeedCommand intakeFeedCommand = new IntakeFeedCommand(intakeSubsystem, shooterSubsystem);
  private final FeedAuto feedAuto = new FeedAuto(intakeSubsystem);

  // Shooter
  private final ShooterShootCommand shooterShootCommand = new ShooterShootCommand(shooterSubsystem);
  private final ShooterInCommand shooterInCommand = new ShooterInCommand(shooterSubsystem);
  private final ShooterAmpCommand shooterAmpCommand = new ShooterAmpCommand(shooterSubsystem);
  private final AutoShootCommand autoShootCommand = new AutoShootCommand(intakeSubsystem, shooterSubsystem);

  // Pivot
  private final PivotTriggerCommand pivotTriggerCommand = new PivotTriggerCommand(pivotSubsystem);
  private final stopPivotCommand stopPivotCommand = new stopPivotCommand(pivotSubsystem);
  private final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

  private final Colors colors = new Colors();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    
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
    NamedCommands.registerCommand("Pivot Far", pivotSubsystem.setPivotShootStageCommand().withTimeout(1.5));

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Nothing", new RunCommand(() -> {}));

    configureBindings();
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

    TelopDrive_wVisionAlign closedFieldRel_wVision = new TelopDrive_wVisionAlign(
      drivebase, 
      () -> HeadingCorrection() * MathUtil.applyDeadband(-rightJoystick.getY(), 0.075),
      () -> HeadingCorrection() * MathUtil.applyDeadband(-rightJoystick.getX(), 0.075),
      () -> MathUtil.applyDeadband(-leftJoystick.getX() * 0.8, 0.075), 
      () -> true, 
      visionSubsystem
    );

    drivebase.setDefaultCommand(closedFieldRel_wVision);
  }

  private void configureDashboard() {
    final var pivotTab = Shuffleboard.getTab("Pivot");
    pivotSubsystem.addDashboardWidgets(pivotTab.getLayout("Pivot", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(5, 4));

    final var shooterAndIntakeTab = Shuffleboard.getTab("Shooter & Intake");
    shooterSubsystem.addDashboardWidgets(shooterAndIntakeTab.getLayout("Shooter", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(3, 3));
    intakeSubsystem.addDashboardWidgets(shooterAndIntakeTab.getLayout("Intake", BuiltInLayouts.kGrid).withPosition(5, 0).withSize(3, 3));
    final var climberTab = Shuffleboard.getTab("Climber");
    climberSubsystem.addDashboardWidgets(climberTab.getLayout("Climber", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(4, 3));

    driverTab.add("Auto", autoChooser).withPosition(3, 0).withSize(3, 2);

    /**driverTab.add(new HttpCamera("photonvision_Port_1184_Output_MJPEG_Server", "http://10.95.76.11:1184"))
      .withWidget(BuiltInWidgets.kCameraStream)
      .withProperties(Map.of("showCrosshair", true, "showControls", true))
      .withSize(4, 5).withPosition(3, 0);*/

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
  
    leftJoystick.button(1).whileTrue(new PhotonAlignCommand(
      photonCamera, drivebase, () -> visionSubsystem.getCurrentPose(), 
      () -> HeadingCorrection() * MathUtil.applyDeadband(-rightJoystick.getY(), 0.075),
      () -> HeadingCorrection() * MathUtil.applyDeadband(-rightJoystick.getX(), 0.075)));


    rightJoystick.button(1)
    .whileTrue(new InstantCommand(() -> drivebase.setAreWeAiming(true)))
    .onFalse(new InstantCommand(() -> drivebase.setAreWeAiming(false)));

    //leftJoystick.button(8).onTrue(new InstantCommand(drivebase::setGyroOffset));
    //rightJoystick.button(7).onTrue(new InstantCommand(() -> intakeSubsystem.setBrake()));
    //rightJoystick.button(8).onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeCoast()));
    //rightJoystick.button(9).onTrue(new InstantCommand(() -> pivotSubsystem.setBrake()));
    //rightJoystick.button(10).onTrue(new InstantCommand(() -> pivotSubsystem.setCoast()));

    leftJoystick.povUp().whileTrue(climberSubsystem.raiseLeftArmCommand());
    rightJoystick.povUp().whileTrue(climberSubsystem.raiseRightArmCommand());
    leftJoystick.povDown().whileTrue(climberSubsystem.lower_LEFT_ArmCommand());
    rightJoystick.povDown().whileTrue(climberSubsystem.lower_RIGHT_ArmCommand());

    xboxControllerCommand.povUp().whileTrue(climberSubsystem.raise_BOTH_ArmCommand());
    xboxControllerCommand.povDown().whileTrue(climberSubsystem.lower_BOTH_ArmCommand());

    //leftJoystick.button(9).whileTrue(new InstantCommand(() -> climberSubsystem.lowerLeftArmOVERRIDE()));/* */
    //leftJoystick.button(10).whileTrue(new InstantCommand(() -> climberSubsystem.lowerRightArmOVERRIDE()));
    rightJoystick.button(1)
    .whileTrue(new InstantCommand(() -> climberSubsystem.fastLower = true).alongWith(new InstantCommand(() -> climberSubsystem.slowRaise = true)))
    .onFalse(new InstantCommand(() -> climberSubsystem.fastLower = false).alongWith(new InstantCommand(() -> climberSubsystem.slowRaise = false)));


    leftJoystick.povDown().and(rightJoystick.povDown()).whileTrue(climberSubsystem.lower_BOTH_ArmCommand());
    leftJoystick.povUp().and(rightJoystick.povUp()).whileTrue(climberSubsystem.raise_BOTH_ArmCommand());


    /**xboxControllerCommand.b().whileTrue(pivotSubsystem.setPivotShootSpeakerCommand().alongWith(new InstantCommand(() -> blinkin.setCustomColor(colors.c1E2E_blend2Blck))));
    xboxControllerCommand.x().whileTrue(pivotSubsystem.setPivotIntakeCommand());
    xboxControllerCommand.y().whileTrue(pivotSubsystem.setPivot_Finish_AMPCommand());
    xboxControllerCommand.a().whileTrue(pivotSubsystem.setPivotShootStageCommand());*/

    //xboxControllerCommand.a().whileTrue(photonAlignCommand);

    new Trigger(intakeSubsystem::hasNoteRAW).and(xboxControllerCommand.rightTrigger())
    .whileTrue(new InstantCommand(() -> xboxController.setRumble(RumbleType.kBothRumble, 0.75)).alongWith(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Stobe_Gold))))
    .onFalse(new InstantCommand(() -> xboxController.setRumble(RumbleType.kBothRumble, 0.0)).alongWith(new InstantCommand(() -> blinkin.setCustomColor(colors.fixPal_Breath_Blue))));
    
    xboxControllerCommand.rightBumper().whileTrue(intakeOutCommand);

    xboxControllerCommand.leftBumper().whileTrue(shooterAmpCommand);

    xboxControllerCommand.leftTrigger().whileTrue(shooterShootCommand);

    //xboxControllerCommand.leftTrigger().and(xboxControllerCommand.rightTrigger()).whileTrue(intakeFeedCommand);

    xboxControllerCommand.rightTrigger().whileTrue(intakeInCommand);

    xboxControllerCommand.povUp().whileTrue(intakeFeedCommand);
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

