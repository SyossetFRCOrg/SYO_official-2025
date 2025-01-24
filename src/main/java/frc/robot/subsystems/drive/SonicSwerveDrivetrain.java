package frc.robot.subsystems.drive;

import java.util.Arrays;

import com.moandjiezana.toml.Toml;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import jakarta.validation.constraints.NotNull;

/**
 * Swerve Drivetrain for use in the Supersonics 2025 Robot
 */
public class SonicSwerveDrivetrain extends Drivetrain {
    private final int numModules;

    private final GyroSensor gyroSensor;
    private final SwerveModule[] modules;

    // Temporary, move this to a robot constants class later
    private final SwerveDriveKinematics kinematics;

    public SonicSwerveDrivetrain(Toml toml) {
        Toml defaultModule = toml.getTable("default_module");
        SwerveModule[] modules = toml.getTables("modules").stream().map(t -> SwerveModule.create(t, defaultModule)).toArray(SwerveModule[]::new);
        
        this.modules = modules;
        setName(toml.getString("name"));
        
        kinematics = new SwerveDriveKinematics(Arrays.stream(modules).map(m -> m.getPosition()).toArray(Translation2d[]::new));
        numModules = modules.length;

        for (var module : modules) {
            addChild(module.getName(), module);
        }

        gyroSensor = new NavXGyroSensor();
        addChild("Gyro Sensor", gyroSensor);
    }

    public SonicSwerveDrivetrain(@NotNull SwerveModule[] modules) {
        // validate entered modules
        for (var module : modules)
            if (module == null)
                throw new NullPointerException("Provided SwerveModule has a value of null");
        
        this.modules = modules;

        kinematics = new SwerveDriveKinematics(Arrays.stream(modules).map(m -> m.getPosition()).toArray(Translation2d[]::new));
        numModules = modules.length;

        for (var module : modules) {
            addChild(getName(), module);
        }

        gyroSensor = new NavXGyroSensor();
    }

    @Override
    public void setSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < numModules; i++) {
            states[i].optimize(modules[i].getAngle());
            modules[i].setTargetClosed(states[i]);
        }
    }

    @Override
    public void periodic() {
        for (var module: modules) {
            module.periodic();
        }
    }

    @Override
    public Rotation3d getAngle() {
        return gyroSensor.getAngle();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Roll", () -> gyroSensor.getAngle().getX(), null);
        builder.addDoubleProperty("Pitch", () -> gyroSensor.getAngle().getY(), null);
        builder.addDoubleProperty("Yaw", () -> gyroSensor.getAngle().getZ(), null);
    }
}
