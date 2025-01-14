package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class GenericSwerveModule implements SwerveModule {
    private final DriveModule drive;
    private final TurnModule turn;

    public GenericSwerveModule(DriveModule drive, TurnModule turn) {
        this.drive = drive;
        this.turn = turn;
    }

    @Override
    public void periodic() {
        drive.periodic();
        turn.periodic();
    }

    @Override
    public void setTargetClosed(Translation2d translation) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTargetClosed'");
    }

    @Override
    public void setTargetClosed(SwerveModuleState state) {
        // state.optimize(turn.getAngle());
        drive.setSpeed(state.speedMetersPerSecond);
        turn.setTarget(state.angle);
    }
    
}
