package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Individual module for a swerve subsystem, representing one wheel and the two motors (drive + steer) associated with it
 */
public interface SwerveModule {
    /**
     * <p> Periodic update for the given swerve module, called on every periodic update for the drivetrain </p>
     */
    public void periodic();

    public Rotation2d getAngle();

    /**
     * <p>Set the target speeds based off of a tranlsation vector</p>
     * <p>Operates on an <b>closed loop</p>, meaning that the outputs of driving <b>are</b> taken into account.
     * Suitable for Autonomous movements </p>
     * @param translation Translation vector of the swerve module. The length of the vector is the velocity in m/s
     */
    public void setTargetClosed(Translation2d translation);
    
    /**
     * <p>Set the target speeds based off of a provided {@link SwerveModuleState}</p>
     * <p>Operates on an <b>closed loop</p>, meaning that the outputs of driving <b>are</b> taken into account.
     * Suitable for Autonomous movements </p>
     * @param state The desired {@link SwerveModuleState}
     */
    public void setTargetClosed(SwerveModuleState state);

    public void setDriveOpen(double metersPerSec);
    
    public void setTurnOpen(double radPerSec);
}
