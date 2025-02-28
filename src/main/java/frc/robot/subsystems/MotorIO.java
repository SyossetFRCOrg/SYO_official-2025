package frc.robot.subsystems;

public interface MotorIO {
    public static class Inputs {
        public boolean connected;
        public double positionRad;
        public double velocityRadPerSec;
        public double appliedVolts;
        public double currentAmps;
    }

    public void updateInputs(Inputs inputs);

    public void setVoltage(double voltage);
    public void setVelocity(double velocity);

    public void setSetpoint(double rad);
    public boolean atSetpoint();
    public void resetPosition(double rad);
}
