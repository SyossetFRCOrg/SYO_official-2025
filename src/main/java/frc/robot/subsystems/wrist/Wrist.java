package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO wristIO;

  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final Debouncer aimedDebounce = new Debouncer(0.4);

  private static final HashMap<SystemState, LoggedTunableNumber> positions = initializePositions();

  public enum WantedState {
        STOW,
        L1,
        L2,
        L3,
        L4,
        L2L3ALGAE,
        L3L4ALGAE,
        INTAKE
    }

    public enum SystemState {
        IN_STOW,
        L1_ING,
        L2_ING,
        L3_ING,
        L4_ING,
        L2L3ALGAE_ING,
        L3L4ALGAE_ING,
        INTAKING
    }
    private WantedState wantedState = WantedState.STOW;
    private SystemState systemState = SystemState.IN_STOW;

  private static final HashMap<SystemState, LoggedTunableNumber> initializePositions() {
    var map = new HashMap<SystemState, LoggedTunableNumber>();
    map.put(SystemState.IN_STOW, new LoggedTunableNumber("Wrist/StowPosition", 0));
    map.put(SystemState.INTAKING, new LoggedTunableNumber("Wrist/IntakePosition", 1.5));

    map.put(SystemState.L2L3ALGAE_ING, new LoggedTunableNumber("Wrist/L2L3AlgaePosition", 2.4));
    map.put(SystemState.L3L4ALGAE_ING, map.get(SystemState.L2L3ALGAE_ING));

    map.put(SystemState.L1_ING, new LoggedTunableNumber("Wrist/L1Position", 1.8));
    map.put(SystemState.L2_ING, new LoggedTunableNumber("Wrist/L2Position", 3.2));
    map.put(SystemState.L3_ING, new LoggedTunableNumber("Wrist/L3Position", 3.2));
    map.put(SystemState.L4_ING, new LoggedTunableNumber("Wrist/L4Position", 3.2));

    // map.put(SystemState.L1PREPARE, map.get(SystemState.L1));
    // map.put(SystemState.L2PREPARE, map.get(SuperState.L2));
    // map.put(SystemState.L3PREPARE, map.get(SuperState.L3));
    // map.put(SystemState.L4PREPARE, map.get(SuperState.L4));

    // map.put(SystemState.INTAKEPREPARE, map.get(SystemState.STOW));
    // map.put(SystemState.INTAKELOW, map.get(SystemState.INTAKE));
    // map.put(SystemState.INTAKELOWPREPARE, map.get(SystemState.INTAKE));

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

    SystemState newState = handleStateTransitions();

    if (newState != systemState) {
      Logger.recordOutput("Shooter/SystemState", newState.toString());
      systemState = newState;
    }
    
    if (RobotState.getInstance().isAboveL1() && RobotState.getInstance().isWristCanMove()) {
      switch (systemState) {
        case IN_STOW -> handleStow();
        case L1_ING -> handleL1();
        case L2_ING -> handleL2();
        case L3_ING -> handleL3();
        case L4_ING -> handleL4();
        case L2L3ALGAE_ING -> handleL2L3Algae();
        case L3L4ALGAE_ING -> handleL3L4Algae();
        case INTAKING -> handleIntaking();
        default -> handleStow();
      }
    }
    else
    {
      handleStow();
    }
  }
  public void handleStow() {
    wristIO.runPosition(positions.get(SystemState.IN_STOW).get());
    }

    public void handleL1() {
    wristIO.runPosition(positions.get(SystemState.L1_ING).get());
    }

    public void handleL2() {
    wristIO.runPosition(positions.get(SystemState.L2_ING).get());
    }

    public void handleL3() {
    wristIO.runPosition(positions.get(SystemState.L3_ING).get());
    }

    public void handleL4() {
    wristIO.runPosition(positions.get(SystemState.L4_ING).get());
    }

    public void handleL2L3Algae() {
    wristIO.runPosition(positions.get(SystemState.L2L3ALGAE_ING).get());
    }

    public void handleL3L4Algae() {
    wristIO.runPosition(positions.get(SystemState.L3L4ALGAE_ING).get());
    }

    public void handleIntaking() {
    wristIO.runPosition(positions.get(SystemState.INTAKING).get());
    }

    private SystemState handleStateTransitions() {
      return switch (wantedState) {
        case STOW -> SystemState.IN_STOW;
        case L1 -> SystemState.L1_ING;
        case L2 -> SystemState.L2_ING;
        case L3 -> SystemState.L3_ING;
        case L4 -> SystemState.L4_ING;
        case L2L3ALGAE -> SystemState.L2L3ALGAE_ING;
        case L3L4ALGAE -> SystemState.L3L4ALGAE_ING;
        case INTAKE -> SystemState.INTAKING;
        default -> SystemState.IN_STOW;
      };
    }



    public void resetPosition(double posRads) {
    wristIO.resetPosition(posRads);
  }

  public boolean atL1() {
    return aimedDebounce.calculate(
      MathUtil.isNear(positions.get(SystemState.L1_ING).get(), inputs.positionRad, 0.05 /*0.106 rad*/)
        && Math.abs(inputs.velocityRadPerSec) < 0.1);
  }
  
  public boolean atL2() {
    return aimedDebounce.calculate(
      MathUtil.isNear(positions.get(SystemState.L1_ING).get(), inputs.positionRad, 0.05 /*0.106 rad*/)
        && Math.abs(inputs.velocityRadPerSec) < 0.1);
  }
  
  public boolean atL3() {
    return aimedDebounce.calculate(
      MathUtil.isNear(positions.get(SystemState.L1_ING).get(), inputs.positionRad, 0.05 /*0.106 rad*/)
        && Math.abs(inputs.velocityRadPerSec) < 0.1);
  }
  
  public boolean atL4() {
    return aimedDebounce.calculate(
      MathUtil.isNear(positions.get(SystemState.L1_ING).get(), inputs.positionRad, 0.05 /*0.106 rad*/)
        && Math.abs(inputs.velocityRadPerSec) < 0.1);
  }

  public double getPosition() {
    return inputs.positionRad;
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }
}
