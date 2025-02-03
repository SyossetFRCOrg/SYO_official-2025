package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorStructure extends SubsystemBase {
    private final Elevator elevator = new Elevator();
    private final CoralArm coralArm = new CoralArm();
    
    public ElevatorStructure() {
        coralArm.setDefaultCommand(coralArm.new Hover());
        elevator.setDefaultCommand(elevator.new Hover());
    }

    public Command getMoveElevator(double velocity) {
        return elevator.new SetVelocity(velocity);
    }

    public Command getResetElevator() {
        return elevator.new ResetPosition();
    }

    public Command getResetArm() {
        return coralArm.new ResetPosition();
    }

    public Command getMoveArm(double velocity) {
        return coralArm.new SetVelocity(velocity);
    }

    public class PositionCommand extends StructureCommand {
        public PositionCommand(double armPos, double elevatorPos) {
            super(
                // command(coralArm, coralArm.new SetPosition(armPos))
                // command(elevator, elevator.new SetPosition(elevatorPos))
            );
        }
    }

    // TODO find values
    public final Command coralIntake = Commands.sequence(
        new PositionCommand(-7.86, 2.0),  
        new PositionCommand(-7.86, -1.64),
        new PositionCommand(-4.0, 2.0)
    );
}
