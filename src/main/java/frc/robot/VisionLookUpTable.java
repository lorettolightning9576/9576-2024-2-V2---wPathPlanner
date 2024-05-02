package frc.robot;

public class VisionLookUpTable {
    ShooterConfig shooterConfig;

    private static VisionLookUpTable instance = new VisionLookUpTable();

    public static VisionLookUpTable getInstance() {
        return instance;
    }

    public VisionLookUpTable() {
        shooterConfig = new ShooterConfig();
        shooterConfig.getShooterConfigs().add(new ShooterPreset(0, 0, 0));
    }

}
