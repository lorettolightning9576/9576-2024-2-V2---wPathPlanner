package frc.robot.commands.swervedrive.drivebase;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;

public class AimAtTag extends Command{

    private final PhotonCamera photonCamera;
    private final SwerveSubsystem swerve;

    private double Linear_P = 0.1;
    private double Linear_D = 0.0;

    private double Angular_P = 0.1;
    private double Angular_D = 0.1;

    private PIDController forwaPidController = new PIDController(Linear_D, 0.0, Linear_D);
    private PIDController turnController = new PIDController(Angular_P, 0.0, Angular_D);

    private boolean hasTargets = false;

    private double rotationSpeed = 0.0;

    private final DoubleSupplier vX;
    private final DoubleSupplier vY;
    private final DoubleSupplier omega;
    private final BooleanSupplier driveMode;
    private final SwerveController controller;

    public AimAtTag(PhotonCamera m_PhotonCamera, SwerveSubsystem m_SwerveSubsystem, DoubleSupplier m_vX, DoubleSupplier m_vY, DoubleSupplier m_omega, BooleanSupplier m_driveMode) {
        this.photonCamera = m_PhotonCamera;
        this.swerve = m_SwerveSubsystem;
        this.vX = m_vX;
        this.vY = m_vY;
        this.omega = m_omega;
        this.driveMode = m_driveMode;
        this.controller = swerve.getSwerveController();
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double xVelocity = Math.pow(vX.getAsDouble(), 3);
        double yVelocity = Math.pow(vY.getAsDouble(), 3);
        double angVelocity = Math.pow(omega.getAsDouble(), 3);

        var results = photonCamera.getLatestResult();

        if (results.hasTargets()) {
            hasTargets = true;
            rotationSpeed = -turnController.calculate(results.getBestTarget().getYaw(), 0);
            SmartDashboard.putNumber("Vision Speed", -turnController.calculate(results.getBestTarget().getYaw(), 0));
        } else {
            hasTargets = false;
            rotationSpeed = angVelocity * controller.config.maxAngularVelocity;
        }

        swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed), 
            rotationSpeed,
            driveMode.getAsBoolean()
        );

        //swerve.drive(new Translation2d(xVelocity * swerve));
    }

    @Override
    public void end(boolean interrupted) {
      swerve.drive(new Translation2d(0 * swerve.maximumSpeed, 0 * swerve.maximumSpeed), 
        0 * controller.config.maxAngularVelocity,
        driveMode.getAsBoolean());
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
