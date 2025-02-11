package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO wristIO;

  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private static final HashMap<SuperState, LoggedTunableNumber> positions = initializePositions();

  private static final HashMap<SuperState, LoggedTunableNumber> initializePositions() {
    var map = new HashMap<SuperState, LoggedTunableNumber>();
    map.put(SuperState.STOW, new LoggedTunableNumber("Wrist/StowPosition", 0));
    map.put(SuperState.INTAKE, new LoggedTunableNumber("Wrist/IntakePosition", -.8));
    map.put(SuperState.L1, new LoggedTunableNumber("Wrist/L1Position", -1.7));
    map.put(SuperState.L2, new LoggedTunableNumber("Wrist/L2Position", -1.7));
    map.put(SuperState.L3, new LoggedTunableNumber("Wrist/L3Position", -1.7));
    map.put(SuperState.L4, new LoggedTunableNumber("Wrist/L4Position", -1.7));

    map.put(SuperState.L1PREPARE, map.get(SuperState.L1));
    map.put(SuperState.L2PREPARE, map.get(SuperState.L2));
    map.put(SuperState.L3PREPARE, map.get(SuperState.L3));
    map.put(SuperState.L4PREPARE, map.get(SuperState.L4));

    map.put(SuperState.INTAKEPREPARE, map.get(SuperState.STOW));

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
    if (positions.containsKey(Superstructure.getCurrentState())) {
      position = positions.get(Superstructure.getCurrentState()).get();
    }

    // for wrist, everything is in radians
    if (RobotState.getInstance().isWristCanMove()) wristIO.runPosition(position);
    else wristIO.runPosition(positions.get(SuperState.STOW).get());
  }

  public boolean atSetPoint(SuperState setpointState) {

    if (positions.containsKey(setpointState)) position = positions.get(setpointState).get();
    return MathUtil.isNear(position, inputs.positionRad, 0.105 /*rad*/);
  }
}
