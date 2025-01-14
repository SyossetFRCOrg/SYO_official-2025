package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import jakarta.validation.constraints.NotNull;

/**
 * Swerve Drivetrain for use in the Supersonics 2025 Robot
 */
public class SonicSwerveDrivetrain extends Drivetrain {
    private final int numModules;

    private final SwerveModule[] modules;

    // Temporary, move this to a robot constants class later
    private final SwerveDriveKinematics kinematics;

    public SonicSwerveDrivetrain(@NotNull SwerveDriveKinematics kinematics, @NotNull SwerveModule[] modules) {
        // validate entered modules
        for (var module : modules)
            if (module == null)
                throw new NullPointerException("Provided SwerveModule has a value of null");

        // validate kinematics
        if (kinematics.getModules().length != modules.length)
            throw new IllegalArgumentException("Kinematics must have an equal number of modules as provided in the constructor");

        this.modules = modules;
        this.kinematics = kinematics;

        numModules = modules.length;
    }

    @Override
    public void setSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < numModules; i++) {
            modules[i].periodic();
            modules[i].setTargetClosed(states[i]);
        }
    }
}
