package frc.robot;

public class ShooterPreset implements Comparable<ShooterPreset>{
    private double pivotAngle;
    private double shooterSpeed;
    private double distance;

    public ShooterPreset(
        double m_pivotAngle,
        double m_shooterSpeed,
        double m_distance) {
            this.pivotAngle = m_pivotAngle;
            this.shooterSpeed = m_shooterSpeed;
            this.distance = m_distance;
    }

    public double getPivotAngle() {
        return pivotAngle;
    }

    public double getShooterSpeed() {
        return shooterSpeed;
    }

    public double getDistance() {
        return distance;
    }

    public void setPivotAngle(double m_pivotAngle) {
        this.pivotAngle = m_pivotAngle;
    }

    public void setShooterSpeed(double m_shooterSpeed) {
        this.shooterSpeed = m_shooterSpeed;
    }

    public void setDistance(double m_distance) {
        this.distance = m_distance;
    }

    @Override
    public int compareTo(ShooterPreset pPreset) {
        return Double.compare(this.getDistance(), pPreset.getDistance());
    }
}
