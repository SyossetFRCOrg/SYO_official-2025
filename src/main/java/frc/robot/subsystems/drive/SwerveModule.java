package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Individual module for a swerve subsystem, representing one wheel and the two motors (drive + steer) associated with it
 */
public abstract class SwerveModule extends SubsystemBase {
    public abstract Rotation2d getAngle();

    /**
     * <p>Set the target speeds based off of a tranlsation vector</p>
     * <p>Operates on an <b>closed loop</p>, meaning that the outputs of driving <b>are</b> taken into account.
     * Suitable for Autonomous movements </p>
     * @param translation Translation vector of the swerve module. The length of the vector is the velocity in m/s
     */
    public abstract void setTargetClosed(Translation2d translation);
    
    /**
     * <p>Set the target speeds based off of a provided {@link SwerveModuleState}</p>
     * <p>Operates on an <b>closed loop</p>, meaning that the outputs of driving <b>are</b> taken into account.
     * Suitable for Autonomous movements </p>
     * @param state The desired {@link SwerveModuleState}
     */
    public abstract void setTargetClosed(SwerveModuleState state);

    public abstract void setDriveOpen(double metersPerSec);
    
    public abstract void setTurnOpen(double radPerSec);
}
