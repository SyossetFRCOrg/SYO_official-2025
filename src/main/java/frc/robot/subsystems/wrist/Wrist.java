package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.elevator.Elevator.Substate;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO wristIO;

  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final Debouncer aimedDebounce = new Debouncer(.1);

  // private static final HashMap<SuperState, LoggedTunableNumber> positions = initializePositions();

  // private static final HashMap<SuperState, LoggedTunableNumber> initializePositions() {
  //   var map = new HashMap<SuperState, LoggedTunableNumber>();
  //   map.put(SuperState.STOW, new LoggedTunableNumber("Wrist/StowPosition", 0));
  //   map.put(SuperState.INTAKE, new LoggedTunableNumber("Wrist/IntakePosition", 1.4));

  //   map.put(SuperState.L2L3ALGAE, new LoggedTunableNumber("Wrist/L2L3AlgaePosition", 2.4));
  //   map.put(SuperState.L3L4ALGAE, map.get(SuperState.L2L3ALGAE));

  //   map.put(SuperState.L1, new LoggedTunableNumber("Wrist/L1Position", 2.2));
  //   map.put(SuperState.L2, new LoggedTunableNumber("Wrist/L2Position", 3.2));
  //   map.put(SuperState.L3, new LoggedTunableNumber("Wrist/L3Position", 3.2));
  //   map.put(SuperState.L4, new LoggedTunableNumber("Wrist/L4Position", 3.58));

  //   map.put(SuperState.L1PREPARE, map.get(SuperState.L1));
  //   map.put(SuperState.L2PREPARE, map.get(SuperState.L2));
  //   map.put(SuperState.L3PREPARE, map.get(SuperState.L3));
  //   map.put(SuperState.L4PREPARE, map.get(SuperState.L4));

  //   map.put(SuperState.INTAKEPREPARE, map.get(SuperState.STOW));
  //   map.put(SuperState.INTAKELOW, map.get(SuperState.INTAKE));
  //   map.put(SuperState.INTAKELOWPREPARE, map.get(SuperState.INTAKE));

  //   return map;
  // }

  public enum Target {
    STOW,
    INTAKE,
    INTAKELOW,
    L1,
    L2,
    L3,
    L4,
    L2L3ALGAE,
    L3L4ALGAE
  }

  public enum Substate {
    STOPPED,
    MOVING_TO_TARGET
  }
  private static final HashMap<Target, LoggedTunableNumber> positions = initializePositions();
  private Substate currentState;
  private Substate desiredState;
  private Target target;

  private static final HashMap<Target, LoggedTunableNumber> initializePositions() {
    var map = new HashMap<Target, LoggedTunableNumber>();

    map.put(Target.STOW, new LoggedTunableNumber("Wrist/StowPosition", 0));
    map.put(Target.INTAKE, new LoggedTunableNumber("Wrist/IntakePosition", 1.4));
    map.put(Target.INTAKELOW, new LoggedTunableNumber("Wrist/LOWIntakePosition", 2.4));
    map.put(Target.L1, new LoggedTunableNumber("Wrist/L1Position", 2.2));
    map.put(Target.L2, new LoggedTunableNumber("Wrist/L2Position", 3.2));
    map.put(Target.L3, new LoggedTunableNumber("Wrist/L3Position", 3.2));
    map.put(Target.L4, new LoggedTunableNumber("Wrist/L4Position", 3.58));
    
    map.put(Target.L2L3ALGAE, new LoggedTunableNumber("Wrist/L2L3A", 2.4));
    map.put(
        Target.L3L4ALGAE,
        new LoggedTunableNumber("Wrist/L3L4A", map.get(Target.L2L3ALGAE).get()));
    return map;
  }

  private double position = 0;

  public Wrist(WristIO wristIO) {
    this.wristIO = wristIO;
    wristIO.resetPosition(0);
    wristIO.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    wristIO.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    wristIO.periodic();

    // if (positions.containsKey(Superstructure.getCurrentSuperState())
    //     && RobotState.getInstance().isAboveL1()) {
    //   position = positions.get(Superstructure.getCurrentSuperState()).get();
    // } else {
    //   position = positions.get(SuperState.STOW).get();
    // }
    if(desiredState != currentState)
    {
      Logger.recordOutput("Wrist/Substate", desiredState.toString());
      currentState = desiredState;
    }

    applyStates();
    // for wrist, everything is in radians
    if (RobotState.getInstance().isWristCanMove()) wristIO.runPosition(position);
    else wristIO.runPosition(positions.get(SuperState.STOW).get());
  }

  private void applyStates()
  {
    switch(currentState)
    {
      case STOPPED:
        wristIO.stop();
        break;
      case MOVING_TO_TARGET:
        moveToTarget();
    }
  }

  public void setDesiredState(Substate desiredState)
  {
    this.desiredState = desiredState;
  }

  public void setTarget(Target target) {
    this.target = target;
  }

  /** Moves wrist to target position */
  public void moveToTarget()
  {
    double desiredPosition = positions.get(target).get();
    wristIO.runPosition(desiredPosition);
  }

  public void resetPosition(double posRads) {
    wristIO.resetPosition(posRads);
  }

  /** checks if wrist is at setpoint */
  public boolean atSetPoint()
  {
    return atSetPoint(target);
  }
  /** checks if wrist is at setpoint given a target */
  public boolean atSetPoint(Target targetPosition)
  {
    double position = positions.get(targetPosition).get();
    return aimedDebounce.calculate(MathUtil.isNear(position, inputs.positionRad, 0.03 /*0.106 rad*/)
               && Math.abs(inputs.velocityRadPerSec) < .1);

  }
  // public boolean atSetPoint(SuperState setpointState) {

  //   if (positions.containsKey(setpointState)) position = positions.get(setpointState).get();
  //   return aimedDebounce.calculate(
  //       MathUtil.isNear(position, inputs.positionRad, 0.03 /*0.106 rad*/)
  //           && Math.abs(inputs.velocityRadPerSec) < .1);
  // }

  public double getPosition() {
    return inputs.positionRad;
  }
}
