package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.swerve.ModuleLimits;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private static final HashMap<SuperState, LoggedTunableNumber> heights = initializeHeights();

  private static final HashMap<SuperState, LoggedTunableNumber> initializeHeights() {
    var map = new HashMap<SuperState, LoggedTunableNumber>();
    map.put(SuperState.STOW, new LoggedTunableNumber("Elevator/StowPosition", 0));
    map.put(SuperState.INTAKE, new LoggedTunableNumber("Elevator/IntakePosition", 3));
    map.put(SuperState.L1, new LoggedTunableNumber("Elevator/L1Position", 4));
    map.put(SuperState.L2, new LoggedTunableNumber("Elevator/L2Position", 5));
    map.put(SuperState.L3, new LoggedTunableNumber("Elevator/L3Position", 6));
    map.put(SuperState.L4, new LoggedTunableNumber("Elevator/L4Position", 7));

    map.put(SuperState.L1PREPARE, map.get(SuperState.L1));
    map.put(SuperState.L2PREPARE, map.get(SuperState.L2));
    map.put(SuperState.L3PREPARE, map.get(SuperState.L3));
    map.put(SuperState.L4PREPARE, map.get(SuperState.L4));

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

    applyStates();
  }

  private void applyStates() {
    var state = Superstructure.getDesiredState();
    if (heights.containsKey(state)) targetHeight = heights.get(state).get();
    io.movetoHeight(targetHeight);
  }

  // /** Returns a command to run a quasistatic test in the specified direction. */
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return sysId.quasistatic(direction);
  // }

  // /** Returns a command to run a dynamic test in the specified direction. */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return sysId.dynamic(direction);
  // }

  /** Check if the intake is close enough to desired state setpoint */
  public boolean atSetPoint() {
    // Make sure the targetHeight is updated
    var state = Superstructure.getDesiredState();
    if (heights.containsKey(state)) targetHeight = heights.get(state).get();
    return MathUtil.isNear(targetHeight, getHeight(), 0.1);
  }

  /** Check if the intake is close enough to the given state setpoint */
  public boolean atSetPoint(SuperState state) {
    var height = targetHeight;
    if (heights.containsKey(state)) height = heights.get(state).get();
    return MathUtil.isNear(height, getHeight(), 0.1);
  }

  /** Returns the current angle of the intake in radians. */
  public double getHeight() {
    return inputs.positionRads;
  }

  public boolean elevatorUp() {
    return getHeight() >= heights.get(SuperState.L2).get();
  }

  public ModuleLimits getModuleLimits() {
    return elevatorUp() && !DriverStation.isAutonomousEnabled()
        ? TunerConstants.moduleLimitsElevatorUp
        : TunerConstants.moduleLimitsFree;
  }

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
