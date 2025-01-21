package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class GenericSwerveModule extends SwerveModule {
    private final DriveModule drive;
    private final TurnModule turn;

    public GenericSwerveModule(DriveModule drive, TurnModule turn) {
        this.drive = drive;
        this.turn = turn;

        addChild(getName(), drive);
        addChild(getName(), turn);
    }

    @Override
    public void periodic() {
        drive.periodic();
        turn.periodic();
    }

    @Override
    public Rotation2d getAngle() {
        return turn.getAngle();
    }

    @Override
    public void setTargetClosed(Translation2d translation) {
        setTargetClosed(new SwerveModuleState(Math.hypot(translation.getX(), translation.getY()), translation.getAngle()));
    }

    @Override
    public void setTargetClosed(SwerveModuleState state) {
        state.optimize(turn.getAngle());
        drive.setSpeed(state.speedMetersPerSecond);
        turn.setTarget(state.angle);
    }

    @Override
    public void setDriveOpen(double metersPerSec) {
        this.drive.setSpeed(metersPerSec);
    }

    @Override
    public void setTurnOpen(double radPerSec) {
        this.turn.setSpeed(radPerSec);
    }
}
