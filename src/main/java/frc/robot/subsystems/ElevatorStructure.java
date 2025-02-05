package frc.robot.subsystems;

import java.util.Set;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ElevatorStructure extends SubsystemBase {
    private final Elevator elevator = new Elevator();
    private final CoralArm coralArm = new CoralArm();

    private State<Event> state;

    public static enum Event {
        INTAKE_PREPARE,
        INTAKE,
        L1_PREPARE,
        L2_PREPARE,
        L3_PREPARE,
        L4_PREPARE,
        L1_SCORE,
        L2_SCORE,
        L3_SCORE,
        L4_SCORE,
        SCORE
    }
    
    public ElevatorStructure() {
        state = NEUTRAL;
        SmartDashboard.putData("Elevator Structure", this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("State", () -> state.toString(), null);
    }

    // These should not be enabled with the drivetrain enabled simultaneously !
    public void debugControls(CommandXboxController controller) {
        elevator.debugControls(controller);
        coralArm.debugControls(controller);

        var ctrlMode = controller.rightTrigger().negate().and(controller.leftTrigger().negate());

        ctrlMode.and(controller.leftBumper()).onTrue(eventCommand(Event.SCORE));
        ctrlMode.and(controller.rightBumper()).onTrue(eventCommand(Event.INTAKE));
        ctrlMode.and(controller.a()).onTrue(eventCommand(Event.L1_PREPARE));
        ctrlMode.and(controller.b()).onTrue(eventCommand(Event.L2_PREPARE));
        ctrlMode.and(controller.x()).onTrue(eventCommand(Event.L3_PREPARE));
        ctrlMode.and(controller.y()).onTrue(eventCommand(Event.L4_PREPARE));

        ctrlMode = controller.rightTrigger().negate().and(controller.leftTrigger());

        ctrlMode.and(controller.a()).onTrue(setHoldCommand());
        ctrlMode.and(controller.b()).onTrue(setNeutralCommand());
    }

    public Command getPositionCommand(double armPos, double elevatorPos) {
        return Commands.parallel(coralArm.getPositionCommand(armPos), elevator.getPositionCommand(elevatorPos));
    }

    public Command eventCommand(Event event) {
        return Commands.defer(() -> state.runEvent(event), Set.of(elevator, coralArm));
    }

    public Command setHoldCommand() {
        return coralArm.getSetKg(0.74).finallyDo(() -> state = HOLD);
    }

    public Command setNeutralCommand() {
        return coralArm.getSetKg(0.44).finallyDo(() -> state = NEUTRAL);
    }
    
    private final State<Event> NEUTRAL = new State<>() {
        @Override
        public Command runEvent(Event event) {
            return switch (event) {
                case INTAKE_PREPARE -> prepareIntake();
                case INTAKE -> prepareIntake().andThen(intake());
                case L1_PREPARE, L2_PREPARE, L3_PREPARE, L4_PREPARE, L1_SCORE, L2_SCORE, L3_SCORE, L4_SCORE
                    -> runEvent(Event.INTAKE).andThen(HOLD.runEvent(event));
                case SCORE -> Commands.none();
                default -> Commands.none();
            };
        }

        @Override
        public String toString() {
            return "NEUTRAL";
        }
    };
    
    private final State<Event> HOLD = new State<>() {
        @Override
        public Command runEvent(Event event) {
            return switch (event) {
                case INTAKE_PREPARE -> prepareIntake();
                case INTAKE -> runEvent(Event.INTAKE_PREPARE).andThen(intake());
                case L1_PREPARE -> prepareL1();
                case L2_PREPARE -> prepareL2();
                case L3_PREPARE -> prepareL3();
                case L4_PREPARE -> prepareL4();
                case L1_SCORE -> runEvent(Event.L1_PREPARE).andThen(L1_READY.runEvent(event));
                case L2_SCORE -> runEvent(Event.L2_PREPARE).andThen(L2_READY.runEvent(event));
                case L3_SCORE -> runEvent(Event.L3_PREPARE).andThen(L3_READY.runEvent(event));
                case L4_SCORE -> runEvent(Event.L4_PREPARE).andThen(L4_READY.runEvent(event));
                case SCORE -> Commands.none();
                default -> Commands.none();
            };
        }

        @Override
        public String toString() {
            return "HOLD";
        }
    };

    private final State<Event> L1_READY = new State<>() {
        @Override
        public Command runEvent(Event event) {
            return switch (event) {
                case L1_PREPARE -> Commands.none();
                case L1_SCORE, SCORE -> scoreL1();
                default -> HOLD.runEvent(event);
            };
        }

        @Override
        public String toString() {
            return "L1_READY";
        }
    };

    private final State<Event> L2_READY = new State<>() {
        @Override
        public Command runEvent(Event event) {
            return switch (event) {
                case L2_PREPARE -> Commands.none();
                case L2_SCORE, SCORE -> scoreL2();
                default -> HOLD.runEvent(event);
            };
        }

        @Override
        public String toString() {
            return "L2_READY";
        }
    };

    private final State<Event> L3_READY = new State<>() {
        @Override
        public Command runEvent(Event event) {
            return switch (event) {
                case L3_PREPARE -> Commands.none();
                case L3_SCORE, SCORE -> scoreL3();
                default -> HOLD.runEvent(event);
            };
        }

        @Override
        public String toString() {
            return "L3_READY";
        }
    };

    private final State<Event> L4_READY = new State<>() {
        @Override
        public Command runEvent(Event event) {
            return switch (event) {
                case L4_PREPARE -> Commands.none();
                case L4_SCORE, SCORE -> scoreL4();
                default -> HOLD.runEvent(event);
            };
        }

        @Override
        public String toString() {
            return "L4_READY";
        }
    };
    

    private final Command prepareIntake() { return getPositionCommand(-Math.PI/2, 5.0); }
    private final Command intake() {
        return Commands.sequence(
            getPositionCommand(-Math.PI/2, 0.0),
            Commands.waitSeconds(0.2),
            getPositionCommand(0.0, 5.0),
            coralArm.getSetKg(0.74)
        ).finallyDo(() -> state = HOLD);
    }

    private final Command prepareL1() { return Commands.none().finallyDo(() -> state = L1_READY); }
    private final Command prepareL2() { return getPositionCommand(1.1, 1.9).finallyDo(() -> state = L2_READY); }
    private final Command prepareL3() { return getPositionCommand(1.1, 11.5).finallyDo(() -> state = L3_READY); }
    private final Command prepareL4() { return getPositionCommand(1.5, 23.0).finallyDo(() -> state = L4_READY); }
    
    private final Command scoreL1() { return Commands.none().andThen(setNeutralCommand()); }
    private final Command scoreL2() { return getPositionCommand(-1.0, -0.1).andThen(setNeutralCommand()); }
    private final Command scoreL3() { return getPositionCommand(-1.0, 10.5).andThen(setNeutralCommand()); }
    private final Command scoreL4() { return getPositionCommand(-1.0, 23.0).andThen(setNeutralCommand()); }
}
