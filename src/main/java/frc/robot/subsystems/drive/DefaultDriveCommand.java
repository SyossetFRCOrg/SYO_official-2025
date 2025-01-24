package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultDriveCommand extends Command {
    private final Drivetrain drivetrain;

    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;
    private final Supplier<Double> omegaSupplier;
    private final double maxVelocity;
    private final double angularVelocity;

    public DefaultDriveCommand(Drivetrain drivetrain, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> omegaSupplier, double maxVelocity, double angularVelocity) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
        this.drivetrain = drivetrain;
        this.maxVelocity = maxVelocity;
        this.angularVelocity = angularVelocity;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.setSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSupplier.get() * maxVelocity, 
                ySupplier.get() * maxVelocity, 
                omegaSupplier.get() * angularVelocity,
                new Rotation2d(drivetrain.getAngle().getMeasureZ())
            )
        );
    }
}
