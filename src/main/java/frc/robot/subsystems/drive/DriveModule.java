package frc.robot.subsystems.drive;

public interface DriveModule {
    public default void periodic() {}

    public void setSpeed(double metersPerSec);
}
