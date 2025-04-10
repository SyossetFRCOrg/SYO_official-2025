package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
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

  private final Debouncer atSetpointDebouncer = new Debouncer(0.3);

  private double heightTolerance = 1; // rad

  DigitalInput zeroLimitSwitch = new DigitalInput(0);

  private static final HashMap<SuperState, LoggedTunableNumber> heights = initializeHeights();

  private static final HashMap<SuperState, LoggedTunableNumber> initializeHeights() {
    var map = new HashMap<SuperState, LoggedTunableNumber>();
    // to be tuned
    map.put(SuperState.STOW, new LoggedTunableNumber("Elevator/StowPosition", 24));
    map.put(SuperState.INTAKE, new LoggedTunableNumber("Elevator/IntakePosition", 27.35));
    map.put(SuperState.INTAKELOW, new LoggedTunableNumber("Elevator/LOWIntakePosition", 25.95));
    map.put(SuperState.L1, new LoggedTunableNumber("Elevator/L1Position", 11));
    map.put(SuperState.L2, new LoggedTunableNumber("Elevator/L2Position", 32.6));
    map.put(SuperState.L3, new LoggedTunableNumber("Elevator/L3Position", 47));
    map.put(SuperState.L4, new LoggedTunableNumber("Elevator/L4Position", 70.7));

    map.put(SuperState.L2L3ALGAE, new LoggedTunableNumber("Elevator/L2L3A", 29.7));
    map.put(
        SuperState.L3L4ALGAE,
        new LoggedTunableNumber("Elevator/L3L4A", map.get(SuperState.L3).get() - 5));

    map.put(SuperState.L1PREPARE, map.get(SuperState.L1));
    map.put(SuperState.L2PREPARE, map.get(SuperState.L2));
    map.put(SuperState.L3PREPARE, map.get(SuperState.L3));
    map.put(SuperState.L4PREPARE, map.get(SuperState.L4));

    map.put(SuperState.INTAKEPREPARE, map.get(SuperState.INTAKE));
    map.put(SuperState.INTAKELOWPREPARE, map.get(SuperState.INTAKELOW));

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
    // io.updateShuffleboard();

    if (inputs.motorType.equals("Sparkmax")) {
      io.periodic();
    }

    Logger.recordOutput("Elevator/AtGoal", atSetPoint());

    applyStates();

    if (!zeroLimitSwitch.get() && RobotState.getInstance().isLimitSwitching()) {
      io.setHeight(0);
    }
    // System.out.println(zeroLimitSwitch.get());
    Logger.recordOutput("Elevator/LimitSwitch", !zeroLimitSwitch.get());

    // modify the Elevator position in RobotState so that the moduleLimits changes so the max
    // acceleration changes
    // depending on the superstate of the superstructure. can technically do this anywhere, but
    // makes most sense in elevator.

    if (getHeight() >= heights.get(SuperState.L4).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(4);
    } else if (getHeight() >= heights.get(SuperState.L3).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(3);
    } else if (getHeight() >= heights.get(SuperState.L2).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(2);
    } else if (getHeight() >= heights.get(SuperState.L1).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(1);
    } else if (getHeight() < heights.get(SuperState.L1).get() - heightTolerance * 1.3) {
      RobotState.getInstance().setElevatorPosition(0);
    }
    // if (Superstructure.getDesiredState() == SuperState.STOW) {
    //   RobotState.getInstance().setElevatorPosition(0);
    // }

    // if (Superstructure.getDesiredState() == SuperState.L1
    //     || Superstructure.getDesiredState() == SuperState.INTAKE) {
    //   RobotState.getInstance().setElevatorPosition(1);
    // }
    // if (Superstructure.getDesiredState() == SuperState.L2) {
    //   RobotState.getInstance().setElevatorPosition(2);
    // }

    // if (Superstructure.getDesiredState() == SuperState.L3
    // // || Superstructure.getDesiredState() == SuperState.AlgaeL2L3
    // // || Superstructure.getDesiredState() == SuperState.AlgaeL3L4
    // ) {
    //   RobotState.getInstance().setElevatorPosition(3);
    // }
    // if (Superstructure.getDesiredState() == SuperState.L4) {
    //   RobotState.getInstance().setElevatorPosition(4);
    // }

    RobotState.getInstance()
        .setAboveL1(getHeight() >= heights.get(SuperState.L1).get() - heightTolerance);
  }

  private void applyStates() {
    var state = Superstructure.getCurrentState();
    if (heights.containsKey(state)) targetHeight = heights.get(state).get();
    RobotState.getInstance()
        .setWristCanMove(getHeight() > heights.get(SuperState.L1).get() - heightTolerance);
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
