package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Debouncer atSetpointDebouncer = new Debouncer(0.5);

  private double heightTolerance = 1.5; // rad

  private static final HashMap<SuperState, LoggedTunableNumber> heights = initializeHeights();

  private static final HashMap<SuperState, LoggedTunableNumber> initializeHeights() {
    var map = new HashMap<SuperState, LoggedTunableNumber>();
    // to be tuned
    map.put(SuperState.STOW, new LoggedTunableNumber("Elevator/StowPosition", 0));
    map.put(SuperState.INTAKE, new LoggedTunableNumber("Elevator/IntakePosition", 66.5));
    map.put(SuperState.L1, new LoggedTunableNumber("Elevator/L1Position", 40));
    map.put(SuperState.L2, new LoggedTunableNumber("Elevator/L2Position", 78));
    map.put(SuperState.L3, new LoggedTunableNumber("Elevator/L3Position", 115));
    map.put(SuperState.L4, new LoggedTunableNumber("Elevator/L4Position", 166));

    map.put(SuperState.L1PREPARE, map.get(SuperState.L1));
    map.put(SuperState.L2PREPARE, map.get(SuperState.L2));
    map.put(SuperState.L3PREPARE, map.get(SuperState.L3));
    map.put(SuperState.L4PREPARE, map.get(SuperState.L4));

    map.put(SuperState.INTAKEPREPARE, map.get(SuperState.INTAKE));

    return map;
  }

  private double targetHeight = 0;

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setBrakeMode(true);

    // // Configure SysId
    // sysId =
    //     new SysIdRoutine(
    //         new SysIdRoutine.Config(
    //             null,
    //             null,
    //             null,
    //             (state) -> Logger.recordOutput("Intake/SysIdState", state.toString())),
    //         new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    io.updateShuffleboard();

    if (inputs.motorType.equals("Sparkmax")) {
      io.periodic();
    }

    Logger.recordOutput("Elevator/AtGoal", atSetPoint());

    applyStates();

    // modify the Elevator position in RobotState so that the moduleLimits changes so the max
    // acceleration changes
    // depending on the superstate of the superstructure. can technically do this anywhere, but
    // makes most sense in elevator.
    switch (Superstructure.getCurrentState()) {
      case STOW:
        RobotState.getInstance().setElevatorPosition(0);
        break;

      case L1PREPARE, L1:
        RobotState.getInstance().setElevatorPosition(1);

      case L2PREPARE, L2, INTAKE, INTAKEPREPARE:
        RobotState.getInstance().setElevatorPosition(2);
        break;

      case L3PREPARE, L3:
        RobotState.getInstance().setElevatorPosition(3);
        break;

      case L4PREPARE, L4:
        RobotState.getInstance().setElevatorPosition(4);
        break;

      case STOPPED:
      default:
        RobotState.getInstance().setElevatorPosition(4);
        break;
    }

    RobotState.getInstance()
        .setAboveL1(getHeight() >= heights.get(SuperState.L1).get() - heightTolerance);
  }

  private void applyStates() {
    var state = Superstructure.getCurrentState();
    if (heights.containsKey(state)) targetHeight = heights.get(state).get();
    RobotState.getInstance().setWristCanMove(getHeight() > heights.get(SuperState.L1).get() - heightTolerance);
    io.movetoHeight(targetHeight);
  }

  /** Check if the height is close enough to desired state setpoint */
  public boolean atSetPoint() {
    // Make sure the targetHeight is updated
    return atSetPoint(Superstructure.getCurrentState());
  }

  /** Check if the height is close enough to the given state setpoint */
  public boolean atSetPoint(SuperState state) {
    var height = targetHeight;
    if (heights.containsKey(state)) height = heights.get(state).get();
    return atSetpointDebouncer.calculate(MathUtil.isNear(height, getHeight(), heightTolerance));
  }

  /** Returns the current angle of the intake in radians. */
  public double getHeight() {
    return inputs.positionRads;
  }

  // public boolean elevatorUp() {
  //   return getHeight() >= heights.get(SuperState.L2).get();
  // }

  /**
   * Resets the angle of the elevator
   *
   * @param positionRads The angle in radians
   */
  public void setHeight(double positionRads) {
    io.setHeight(positionRads);
  }

  /** Stop slam elevator */
  public void stop() {
    io.stop();
  }
}
