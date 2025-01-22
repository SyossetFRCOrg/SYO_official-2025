package frc.robot.subsystems.drive;

import java.util.List;

import com.moandjiezana.toml.Toml;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
     * <p>Set the speed of the chassis based off of the provided {@link ChassisSpeeds}</p>
     * @param speeds target {@link ChassisSpeeds} of the chassis, measured in m/s and rad/s
     */
    public abstract void setSpeeds(ChassisSpeeds speeds);

    public static Drivetrain create(Toml toml) {
        var type = toml.getString("type");
        if (type.equals("sonic_swerve")) {
            return new SonicSwerveDrivetrain(toml);
        } else {
            throw new IllegalArgumentException(String.format("Unsupported Drivetrain type, %s", type));
        }
    }
}