package frc.robot.subsystems;

import org.syofrc.syolib.state.StateMachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ElevatorStructure extends SubsystemBase {
    private final Elevator elevator = new Elevator();
    private final CoralArm coralArm = new CoralArm();
    
    public ElevatorStructure() {
        coralArm.setDefaultCommand(coralArm.new Hover());
        elevator.setDefaultCommand(elevator.new Hover());
    }

    public void bindControls(CommandXboxController controller) {
        controller.x().or(controller.y())
            .whileTrue(elevator.new FreeMove(() -> 
                (controller.rightTrigger().getAsBoolean() ? 0.5 : 1.0) * (
                    (controller.y().getAsBoolean() ? 12.0 : 0.0) +
                    (controller.x().getAsBoolean() ? -12.0 : 0.0)
                )
            ));
            
        controller.a().or(controller.b())
            .whileTrue(coralArm.new FreeMove(() -> 
                (controller.rightTrigger().getAsBoolean() ? 0.5 : 1.0) * (
                    (controller.b().getAsBoolean() ? 2.0 : 0.0) +
                    (controller.a().getAsBoolean() ? -2.0 : 0.0)
                )
            ));

        controller.povLeft().onTrue(elevator.new ResetPosition());
        controller.povRight().onTrue(coralArm.new ResetPosition());
        controller.leftBumper().and(controller.leftTrigger()).onTrue(coralArm.new MoveToPosition(Math.PI / 2));
        controller.rightBumper().and(controller.leftTrigger()).onTrue(coralArm.new MoveToPosition(0));

        controller.leftBumper().and(controller.leftTrigger().negate()).onTrue(elevator.new MoveToPosition(16.0));
        controller.rightBumper().and(controller.leftTrigger().negate()).onTrue(elevator.new MoveToPosition(0));
    }

    public Command getPositionCommand(double armPos, double elevatorPos) {
        return Commands.parallel(coralArm.new MoveToPosition(armPos), elevator.new MoveToPosition(elevatorPos));
    }

    // TODO find values
    public final Command coralIntake = Commands.sequence(
        getPositionCommand(-7.86, 2.0),  
        getPositionCommand(-7.86, -1.64),
        getPositionCommand(-4.0, 2.0)
    );
}
