package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * <p>Subsystem that controls the base chassis and all the movement associated with it</p>
 * <p>Core Modules:</p>
 * <ul>
 *   <li>Movement forward and backward</li>
 *   <li>Rotation left and right</li>
 * </ul>
 * <p>Auxiliary Features:</p>
 *   <li>Omnidirectional movement</li>
 *   <li>Odometry system</li>
 * </p>
 */
public abstract class Drivetrain extends SubsystemBase {
    /**
     * <p>Set the speed of the chassis based off of the dx, dy, and dtheta</p>
     * @param speeds
     */
    public abstract void setSpeeds(Twist2d speeds);
}