package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.sendable.SendableBuilder;

public class NavXGyroSensor extends GyroSensor {
    private final AHRS ahrs;
    private double zeroYaw;
    private boolean init = false;

    public NavXGyroSensor() {
        ahrs = new AHRS(NavXComType.kMXP_SPI);
    }

    @Override
    public Rotation3d getAngle() {
        if (!init) {
            init = true;
            zeroYaw = ahrs.getYaw();
        }

        return new Rotation3d(
            Math.toRadians(ahrs.getRoll()), 
            Math.toRadians(ahrs.getPitch()),
            Math.toRadians(ahrs.getYaw() - zeroYaw)
        );
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Pitch", () -> Math.toRadians(-ahrs.getPitch()), null);
        builder.addDoubleProperty("Roll", () -> Math.toRadians(-ahrs.getRoll()), null);
        builder.addDoubleProperty("Pitch", () -> Math.toRadians(-ahrs.getYaw() + zeroYaw), null);
    }
}
