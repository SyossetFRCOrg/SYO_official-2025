package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;

public class Wrist extends SubsystemBase {
  private final WristIO wristIO;

  private static final HashMap<SuperState, LoggedTunableNumber> positions = initializePositions();

  private static final HashMap<SuperState, LoggedTunableNumber> initializePositions() {
    var map = new HashMap<SuperState, LoggedTunableNumber>();
    map.put(SuperState.STOW, new LoggedTunableNumber("Wrist/StowPosition", 0));
    map.put(SuperState.INTAKE, new LoggedTunableNumber("Wrist/IntakePosition", 0));
    map.put(SuperState.L1, new LoggedTunableNumber("Wrist/L1Position", 0));
    map.put(SuperState.L2, new LoggedTunableNumber("Wrist/L2Position", 0));
    map.put(SuperState.L3, new LoggedTunableNumber("Wrist/L3Position", 0));
    map.put(SuperState.L4, new LoggedTunableNumber("Wrist/L4Position", 0));

    map.put(SuperState.L1PREPARE, map.get(SuperState.L1));
    map.put(SuperState.L2PREPARE, map.get(SuperState.L2));
    map.put(SuperState.L3PREPARE, map.get(SuperState.L3));
    map.put(SuperState.L4PREPARE, map.get(SuperState.L4));

    return map;
  }

  private double position = 0;

  public Wrist(WristIO wristIO) {
    this.wristIO = wristIO;
  }

  @Override
  public void periodic() {
    if (positions.containsKey(Superstructure.getDesiredState())) {
      position = positions.get(Superstructure.getDesiredState()).get();
    }

    wristIO.runPosition(position);
  }
}
