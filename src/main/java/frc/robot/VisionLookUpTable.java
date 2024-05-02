package frc.robot;

public class VisionLookUpTable {
    ShooterConfig shooterConfig;

    private static VisionLookUpTable instance = new VisionLookUpTable();

    public static VisionLookUpTable getInstance() {
        return instance;
    }

    public VisionLookUpTable() {
        shooterConfig = new ShooterConfig();
        shooterConfig.getShooterConfigs().add(new ShooterPreset(40.0, 4500, 1.0));
        shooterConfig.getShooterConfigs().add(new ShooterPreset(34.1, 4500, 1.5));
        shooterConfig.getShooterConfigs().add(new ShooterPreset(30.5, 4500, 2.0));
        shooterConfig.getShooterConfigs().add(new ShooterPreset(29.5, 4500, 2.2));
    }

}
