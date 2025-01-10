package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Individual module for a swerve subsystem, representing one wheel and the two motors (drive + steer) associated with it
 */
public interface SwerveModule {
    /**
     * <p>Set the target speeds based off of a tranlsation vector</p>
     * <p>Operates on an <b>open loop</p>, meaning that the outputs of driving <b>are not</b> taken into account.
     * Suitable for TeleOp Drive</p>
     * @param translation Translation vector of the swerve module. The length of the vector is the velocity in m/s
     */
    public void setTargetOpen(Translation2d translation);
    
    /**
     * <p>Set the target speeds based off of a tranlsation vector</p>
     * <p>Operates on an <b>closed loop</p>, meaning that the outputs of driving <b>are</b> taken into account.
     * Suitable for Autonomous movements </p>
     * @param translation Translation vector of the swerve module. The length of the vector is the velocity in m/s
     */
    public void setTargetClosed(Translation2d translation);
}
