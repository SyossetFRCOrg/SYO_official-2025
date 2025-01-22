package frc.robot.subsystems.drive;

import java.util.Arrays;
import java.util.stream.Collectors;

import com.moandjiezana.toml.Toml;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SonicSwerveModule extends SwerveModule {
    private final Translation2d position;
    private final DriveMotor driveMotor;
    private final TurnMotor turnMotor;
    private final DriveEncoder[] driveEncoders;
    private final TurnEncoder[] turnEncoders;
    private SwerveModuleState targetState;
    
    public SonicSwerveModule(Toml toml) {
        position = new Translation2d(
            toml.getDouble("x", 0.0),
            toml.getDouble("y", 0.0)
        );

        driveMotor = DriveMotor.create(toml.getTable("drive_motor"));
        turnMotor = TurnMotor.create(toml.getTable("turn_motor"));

        driveEncoders = toml.getTables("drive_encoders").stream().map(t -> {
            var type = t.getString("type");
            if (type.equals("sparkmax")) {
                return new SparkMaxDriveMotor.Encoder((SparkMaxDriveMotor)driveMotor);
            } else {
                throw new IllegalArgumentException(String.format("Unsupported Drive Encoder type: %s", type));
            }
        }).toArray(DriveEncoder[]::new);
        
        turnEncoders = toml.getTables("turn_encoders").stream().map(t -> {
            var type = t.getString("type");
            if (type.equals("sparkmax")) {
                return new SparkMaxTurnMotor.Encoder((SparkMaxTurnMotor)turnMotor);
            } else {
                throw new IllegalArgumentException(String.format("Unsupported Drive Encoder type: %s", type));
            }
        }).toArray(TurnEncoder[]::new);
    }

    @Override
    public Rotation2d getAngle() {
        return new Rotation2d(Arrays.stream(turnEncoders).mapToDouble(e -> e.getAngle().getRadians()).average().orElseThrow());
    }

    @Override
    public void setTargetClosed(Translation2d translation) {
        targetState = new SwerveModuleState(translation.getNorm(), translation.getAngle());
    }

    @Override
    public void setTargetClosed(SwerveModuleState state) {
        targetState = state;
    }

    @Override
    public void setDriveOpen(double metersPerSec) {
        driveMotor.setSpeed(metersPerSec);
    }

    @Override
    public void setTurnOpen(double radPerSec) {
        turnMotor.setSpeed(radPerSec);
    }

    @Override
    public Translation2d getPosition() {
        return position;
    }
}
