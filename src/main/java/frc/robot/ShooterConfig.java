package frc.robot;

import java.util.ArrayList;
import java.util.List;

public class ShooterConfig {
    private List<ShooterPreset> shooterConfigs;

    public ShooterConfig() {
        shooterConfigs = new ArrayList<>();
    }

    public ShooterConfig(ArrayList<ShooterPreset> m_ShooterConfigs) {
        this.shooterConfigs = m_ShooterConfigs;
    }

    public List<ShooterPreset> getShooterConfigs() {
        return shooterConfigs;
    }
}
