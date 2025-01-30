package frc.robot.subsystems.drive.swerve;

import java.util.Arrays;
import java.util.function.Supplier;

import com.moandjiezana.toml.Toml;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private final Module[] modules;
    private final SwerveDriveKinematics kinematics;
    private final int numModules;

    public Drivetrain() {
        numModules = 4;

        ModuleIOSpark.Config baseSparkConfig = new ModuleIOSpark.Config();
        baseSparkConfig.drive.idleMode = IdleMode.kBrake;
        baseSparkConfig.drive.motorType = MotorType.kBrushless;
        baseSparkConfig.drive.stallLimit = 50;
        baseSparkConfig.drive.gearRatio = 8.14;
        
        baseSparkConfig.turn.inverted = true;
        baseSparkConfig.turn.idleMode = IdleMode.kBrake;
        baseSparkConfig.turn.motorType = MotorType.kBrushless;
        baseSparkConfig.turn.stallLimit = 20;
        baseSparkConfig.turn.gearRatio = 21.43;

        ModuleIOSpark.Config[] sparkConfigs = new ModuleIOSpark.Config[numModules];
        
        Module.Config baseModuleConfig = new Module.Config();
        baseModuleConfig.wheelRadius = Units.inchesToMeters(1.7);

        Module.Config[] moduleConfigs = new Module.Config[numModules];

        try {
            for (int i = 0; i < numModules; i++) {
                sparkConfigs[i] = (ModuleIOSpark.Config)baseSparkConfig.clone();
            }

            for (int i = 0; i < numModules; i++) {
                moduleConfigs[i] = (Module.Config)baseModuleConfig.clone();
            }
        } catch (CloneNotSupportedException exception) {
            
        }

        sparkConfigs[0].drive.canid = 1;
        sparkConfigs[0].turn.canid = 2;
        sparkConfigs[0].cancoder.canid = 3;
        sparkConfigs[0].cancoder.zeroRotationRad = -3.00;
        
        sparkConfigs[1].drive.canid = 4;
        sparkConfigs[1].turn.canid = 5;
        sparkConfigs[1].cancoder.canid = 6;
        sparkConfigs[1].cancoder.zeroRotationRad = 0.32;
        
        sparkConfigs[2].drive.canid = 7;
        sparkConfigs[2].turn.canid = 8;
        sparkConfigs[2].cancoder.canid = 9;
        sparkConfigs[2].cancoder.zeroRotationRad = 2.11;
        
        sparkConfigs[3].drive.canid = 10;
        sparkConfigs[3].turn.canid = 11;
        sparkConfigs[3].cancoder.canid = 12;
        sparkConfigs[3].cancoder.zeroRotationRad = -1.33;

        moduleConfigs[0].xPos = 3.175;
        moduleConfigs[0].yPos = 3.175;
        moduleConfigs[0].name = "Front Left";
        
        moduleConfigs[1].xPos = 3.175;
        moduleConfigs[1].yPos = -3.175;
        moduleConfigs[1].name = "Front Right";
        
        moduleConfigs[2].xPos = -3.175;
        moduleConfigs[2].yPos = 3.175;
        moduleConfigs[2].name = "Back Left";
        
        moduleConfigs[3].xPos = -3.175;
        moduleConfigs[3].yPos = -3.175;
        moduleConfigs[3].name = "Back Right";

        modules = new Module[numModules];
        for (int i = 0; i < numModules; i++) {
            modules[i] = new Module(moduleConfigs[i], new ModuleIOSpark(sparkConfigs[i]));
        }

        kinematics = new SwerveDriveKinematics(Arrays.stream(moduleConfigs).map(config -> new Translation2d(config.xPos, config.yPos)).toArray(Translation2d[]::new));
    }

    public Drivetrain(Toml toml) {
        var baseModule = toml.getTable("baseModule");
        var moduleTomls = toml.getTables("modules");
        numModules = moduleTomls.size();

        modules = moduleTomls.stream().map(module -> {
            var moduleToml = new Toml(baseModule).read(module);
            var moduleConfig = moduleToml.to(Module.Config.class);
            var sparkConfig = moduleToml.getTable("io").to(ModuleIOSpark.Config.class);

            return new Module(moduleConfig, new ModuleIOSpark(sparkConfig));
        }).toArray(Module[]::new);

        kinematics = new SwerveDriveKinematics(Arrays.stream(modules).map(module -> module.getPos()).toArray(Translation2d[]::new));
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        var states = kinematics.toSwerveModuleStates(speeds);

        for (int i = 0; i < numModules; i++) {
            modules[i].setState(states[i]);
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < numModules; i++) {
            SmartDashboard.putNumber(String.valueOf(i), modules[i].getAngle().getRadians());
        }
    }

    public class DefaultDrive extends Command {
        private final Supplier<Double> xSupplier;
        private final Supplier<Double> ySupplier;
        private final Supplier<Double> omegaSupplier;

        public DefaultDrive(Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> omegaSupplier) {
            this.xSupplier = xSupplier;
            this.ySupplier = ySupplier;
            this.omegaSupplier = omegaSupplier;
            addRequirements(Drivetrain.this);
        }

        @Override
        public void execute() {
            setSpeeds(new ChassisSpeeds(xSupplier.get() * 2.0, ySupplier.get() * 2.0, omegaSupplier.get() * 4.0));
        }
    }
}
