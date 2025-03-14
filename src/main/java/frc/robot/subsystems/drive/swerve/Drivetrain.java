package frc.robot.subsystems.drive.swerve;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.moandjiezana.toml.Toml;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private final Module[] modules;
    private final SwerveDriveKinematics kinematics;
    private final int numModules;

    private final Pigeon2 pigeon =
      new Pigeon2(
          21,
          "rio");
    private StatusSignal<Angle> yaw = pigeon.getYaw();

    private ChassisSpeeds previousChassisSpeeds = new ChassisSpeeds();


    public Drivetrain() {

        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(50);
        pigeon.optimizeBusUtilization();
            

          
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
        sparkConfigs[0].cancoder.zeroRotationRad = -2.908;
        
        sparkConfigs[1].drive.canid = 4;
        sparkConfigs[1].turn.canid = 5;
        sparkConfigs[1].cancoder.canid = 6;
        sparkConfigs[1].cancoder.zeroRotationRad = 0.552;
        
        sparkConfigs[2].drive.canid = 7;
        sparkConfigs[2].turn.canid = 8;
        sparkConfigs[2].cancoder.canid = 9;
        sparkConfigs[2].cancoder.zeroRotationRad = 1.941;
        
        sparkConfigs[3].drive.canid = 10;
        sparkConfigs[3].turn.canid = 11;
        sparkConfigs[3].cancoder.canid = 12;
        sparkConfigs[3].cancoder.zeroRotationRad = -1.588;

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

    public Command setRotation (double degrees){
        return new InstantCommand(() -> pigeon.getConfigurator().setYaw(degrees));
    }

    public void setSpeeds(ChassisSpeeds speeds) {

        
    // the following serves as global translational acceleration limiting

    Translation2d prevSpeedsTranslation = GeomUtil.toTranslation2d(previousChassisSpeeds);
    // GeomUtil.toTranslation2d(getChassisSpeeds());

    Translation2d desiredSpeedsTranslation = GeomUtil.toTranslation2d(speeds);

    Translation2d TranslationDelta = desiredSpeedsTranslation.minus(prevSpeedsTranslation);

    double maxTranslationDeltaPerLoopRatio =
        TranslationDelta
                .getNorm() /*magnitude of difference of current and desired velocity vectors*/
            / ((.75) //max acceleration in m/s^2
             * .02);

    if (maxTranslationDeltaPerLoopRatio > 1) {
      // have to make it so that it approaches prevSpeedsTranslation in a
      // 1/maxTranslationDeltaPerSecRatio ratio,
      // meant to reduce the delta so that we do not hit the tipping point.
      TranslationDelta =
          TranslationDelta.div(
              Math.sqrt(maxTranslationDeltaPerLoopRatio)); // it works, do the math yourself.
    }

    desiredSpeedsTranslation = prevSpeedsTranslation.plus(TranslationDelta);
    speeds.vxMetersPerSecond = desiredSpeedsTranslation.getX();
    speeds.vyMetersPerSecond = desiredSpeedsTranslation.getY();

    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    var states = kinematics.toSwerveModuleStates(discreteSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states,
        3.5 //maximum velocity in m/s
        );


        for (int i = 0; i < numModules; i++) {
            modules[i].setState(states[i]);
        }
        previousChassisSpeeds = discreteSpeeds;
    }

    public Rotation2d getRotation(){
        return Rotation2d.fromDegrees(yaw.getValueAsDouble());
    }

    @Override
    public void periodic() {
        for (int i = 0; i < numModules; i++) {
            SmartDashboard.putNumber(String.valueOf(i), modules[i].getAngle().getRadians());
        }
        yaw = pigeon.getYaw();
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.05);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    
        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;
    
        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
      }
    
      /**
       * Field relative drive command using two joysticks (controlling linear and angular velocities).
       */
      public Command joystickDrive(
          Drivetrain drive,
          DoubleSupplier xSupplier,
          DoubleSupplier ySupplier,
          DoubleSupplier omegaSupplier) {
        return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    
              // Apply rotation deadband
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), 0.05);
    
              // Square rotation value for more precise control
              omega = Math.copySign(omega * omega, omega);
    
              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX()
                          * (5600.0 / 60.0) / ((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)) * 2 * Math.PI * Units.inchesToMeters(1.7),
                      linearVelocity.getY()
                          * (5600.0 / 60.0) / ((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)) * 2 * Math.PI * Units.inchesToMeters(1.7),
                      omega * (5600.0 / 60.0) / ((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)) * Units.inchesToMeters(1.7) / Units.inchesToMeters(Math.sqrt(1800)));
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.setSpeeds(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive);
      }


}
