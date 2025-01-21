package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DriveModule extends SubsystemBase {
    public abstract void setSpeed(double metersPerSec);
    public abstract void setVoltage(double volts);
}
