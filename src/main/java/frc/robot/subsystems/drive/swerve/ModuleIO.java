package frc.robot.subsystems.drive.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
    public static class Inputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public Rotation2d turnZeroRotation = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public Rotation2d turnVelocity = new Rotation2d();
    }

    public void updateInputs(Inputs inputs);

    public void setDriveVoltage(double voltage);
    public void setTurnVoltage(double voltage);
    
    public void setDriveVelocity(double radPerSec);
    public void setTurnVelocity(double radPerSec);

    public void setTurnPosition(double rad);
}
