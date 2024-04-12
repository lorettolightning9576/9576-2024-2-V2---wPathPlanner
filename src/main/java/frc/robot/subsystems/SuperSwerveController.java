package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SuperSwerveController {
    public PIDController headingPID;
    private double kSpinP = 0.015;
    private double kSPinI = 0.00;
    private double kSpinD = 0.00;

    public PIDController turnToHeadingPID;
    private double TurnSpinP =      0.5;
    private double TurnkSpingI =    0.0;
    private double TurnkSpinD =     0.0;

    SwerveSubsystem drivebase;
    Double headingSetpoint = null;

    public SuperSwerveController(SwerveSubsystem m_drivebase) {
        headingPID = new PIDController(kSpinP, kSPinI, kSpinD);
        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        SmartDashboard.putData("HeadingPID", headingPID);

        turnToHeadingPID = new PIDController(TurnSpinP, TurnkSpingI, TurnkSpinD);
        turnToHeadingPID.enableContinuousInput(-Math.PI, Math.PI);
        SmartDashboard.putData("TurnToHeadingPID", turnToHeadingPID);

        this.drivebase = m_drivebase;
        headingSetpoint = drivebase.getPose().getRotation().getDegrees();
    }

    Timer turnTimer = new Timer();

    public void turnTo(SwerveSubsystem swerve, double targetHeading) {
        double targetomega = 0;
        if (headingSetpoint != null) {
            targetomega = turnToHeadingPID.calculate(drivebase.getPose().getRotation().getRadians(), Math.toRadians(targetHeading));

            headingSetpoint = targetHeading;
        }

        swerve.drive(new Translation2d(0, 0), targetomega * swerve.getMaximumAngularVelocity(), true);
        SmartDashboard.putNumber("SuperSwerve.targetOmega", targetomega);
        if (headingSetpoint != null) {
            SmartDashboard.putNumber("SuperSwerve.headingSetpoint", headingSetpoint);
        }
    }

    
}
