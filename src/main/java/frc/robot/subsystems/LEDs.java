package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.SuperState;
import java.util.HashMap;

public class LEDs extends SubsystemBase {

  Spark blinkin;

  double color;

  private static final HashMap<SuperState, Double> colors = initializeColors();

  private static final HashMap<SuperState, Double> initializeColors() {
    var map = new HashMap<SuperState, Double>();
    // to be tuned
    map.put(SuperState.STOPPED, Double.valueOf(-.15));
    map.put(SuperState.STOW, Double.valueOf(.71));

    map.put(SuperState.INTAKE, Double.valueOf(.67));

    map.put(SuperState.INTAKELOW, Double.valueOf(.69));

    map.put(SuperState.INTAKEPREPARE, Double.valueOf(.73));
    map.put(SuperState.INTAKELOWPREPARE, Double.valueOf(.73));

    map.put(SuperState.L1, Double.valueOf(.85));
    map.put(SuperState.L2, map.get(SuperState.L1));
    map.put(SuperState.L3, map.get(SuperState.L1));
    map.put(SuperState.L4, map.get(SuperState.L1));

    map.put(SuperState.L2L3ALGAE, Double.valueOf(.81));
    map.put(SuperState.L3L4ALGAE, Double.valueOf(.81));

    map.put(SuperState.L1PREPARE, Double.valueOf(.91));
    map.put(SuperState.L2PREPARE, map.get(SuperState.L1PREPARE));
    map.put(SuperState.L3PREPARE, map.get(SuperState.L1PREPARE));
    map.put(SuperState.L4PREPARE, map.get(SuperState.L1PREPARE));

    return map;
  }

  public LEDs() {
    blinkin = new Spark(1);
  }

  @Override
  public void periodic() {

    if (colors.containsKey(Superstructure.getCurrentState())) {
      color = colors.get(Superstructure.getCurrentState()).doubleValue();
    } else {
      color = colors.get(SuperState.STOW).doubleValue();
    }

    if (DriverStation.getAlliance().get() == Alliance.Red
        && color == colors.get(SuperState.STOPPED).doubleValue()) {
      color -= .02;
    }

    blinkin.set(color);
  }
}
