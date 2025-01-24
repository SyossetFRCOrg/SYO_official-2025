package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorStructure extends SubsystemBase {
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem coralArm;

    public ElevatorStructure() {
        elevator = new ElevatorSubsystem();
        coralArm = new ArmSubsystem();

        coralArm.setDefaultCommand(coralArm.new Hover());
        elevator.setDefaultCommand(elevator.new Hover());
    }

    public Command getMoveElevator(Supplier<Double> movementSupplier) {
        return elevator.new FreeMove(movementSupplier);
    }

    public Command getMoveArm(Supplier<Double> movementSupplier) {
        return coralArm.new FreeMove(movementSupplier);
    }

    public class PositionCommand extends StructureCommand {
        public PositionCommand(double armPos, double elevatorPos) {
            super(
                command(coralArm, coralArm.new SetPosition(armPos)),
                command(elevator, elevator.new SetPosition(elevatorPos))
            );
        }
    }

    // TODO find values
    public final Command coralIntakePrep = new PositionCommand(0.0, 0.0);
    public final Command coralIntakeDescent = new PositionCommand(0.0, 0.0);
    public final Command coralIntakeAscent = new PositionCommand(0.0, 0.0);
    
    public final Command coralIntake = Commands.sequence(coralIntakePrep, coralIntakeDescent, coralIntakeAscent);
}
