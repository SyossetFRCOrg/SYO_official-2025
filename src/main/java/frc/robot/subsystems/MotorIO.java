package frc.robot.subsystems;

public interface MotorIO {
    public static class Inputs {
        public boolean connected;
        public double positionRad;
        public double velocityRadPerSec;
        public double appliedVolts;
        public double currentAmps;
    }

    public void runSetpoint();

    public void updateInputs(Inputs inputs);
    public double getPosition();

    public void setVoltage(double voltage);
    public void setVelocity(double velocity);

    public void setSetpoint(double rad);
    public void resetPosition(double rad);
}
