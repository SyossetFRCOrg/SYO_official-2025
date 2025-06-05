package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
// import frc.robot.subsystems.climber.ClimberSubsystem;
// import frc.robot.config.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

// import frc.robot.subsystems.shooter.ShooterSubsystem;
// import frc.robot.subsystems.swerve.SwerveSubsystem;
// import frc.robot.subsystems.turret.TurretSubsystem;

public class Superstructure extends SubsystemBase {
  private Drive drive;
  private Elevator elevator;
  private RobotContainer container;
  private Wrist wrist;

  // private ClimberSubsystem climber;

  // private final Timer climberHeadingLockTimer = new Timer();

  // private boolean subwooferShotMode = false;
  // private boolean feedShotMode = false;

  public static enum SuperState {
    // MANUAL,
    INTAKEPREPARE,
    INTAKE,
    INTAKELOW,
    INTAKELOWPREPARE,
    L1,
    L2,
    L3,
    L4,
    L1PREPARE,
    L2PREPARE,
    L3PREPARE,
    L4PREPARE,
    STOPPED,
    STOW,
    L2L3ALGAE,
    L3L4ALGAE
  }

  private static @Getter @Setter SuperState desiredState = SuperState.STOW;
  private static @Getter @Setter SuperState currentState = SuperState.STOW;
  private static SuperState previousState = SuperState.STOW;

  // RobotState.AimingParameters aimingParameters =
  //         new RobotState.AimingParameters(new Rotation2d(), new Rotation2d(), new
  // Translation2d(), 0.0);

  // private static final double CLIMBER_MOTOR_ROTATIONS_CLIMB = 105.0;
  // private static final double CLIMBER_MOTOR_ROTATIONS_CLIMB_SECONDARY = 45.0;
  // private Rotation2d manualTurretSetpoint = new Rotation2d();
  // private Rotation2d manualPitchSetpoint = new Rotation2d();

  public Superstructure(Drive drive, Elevator elevator, Wrist wrist, RobotContainer container) {
    this.drive = drive;
    this.elevator = elevator;
    this.container = container;
    this.wrist = wrist;
  }

  @Override
  public void periodic() {
    // complex logging stuff for later on...? If we want to

    // double percentageOfThreeMetersPerSecond = Math.hypot(
    //                 RobotState.getInstance().getChassisSpeeds().vxMetersPerSecond,
    //                 RobotState.getInstance().getChassisSpeeds().vyMetersPerSecond)
    //         / 3.0;
    // aimingParameters = RobotState.getInstance()
    //         .getAimingParameters(0.6 * percentageOfThreeMetersPerSecond, 0.25 *
    // percentageOfThreeMetersPerSecond);
    currentState = handleStateTransitions();
    applyStates();

    // janky way of logging robotstate values. Robot state @AutoLog doesn't work???
    Logger.recordOutput("RobotState/aboveL1", RobotState.getInstance().isAboveL1());

    Logger.recordOutput(
        "RobotState/elevatorPosition", RobotState.getInstance().getElevatorPosition());

    Logger.recordOutput("RobotState/addingVision", RobotState.getInstance().isAddingVision());

    Logger.recordOutput("RobotState/wristCanMove", RobotState.getInstance().isWristCanMove());

    Logger.recordOutput(
        "RobotState/reefAutoAligning", RobotState.getInstance().isReefAutoAligning());

    Logger.recordOutput("RobotState/reefAutoAiming", RobotState.getInstance().isReefAutoAiming());

    Logger.recordOutput("RobotState/isLSwitching", RobotState.getInstance().isLimitSwitching());
    Logger.recordOutput(
        "RobotState/intakeAutoAiming", RobotState.getInstance().isIntakeAutoAiming());

    if (RobotState.getInstance().getTuningTempPose() != null) {
      Logger.recordOutput(
          "RobotState/tuningTempPose",
          new double[] {
            RobotState.getInstance().getTuningTempPose().getX(),
            RobotState.getInstance().getTuningTempPose().getY(),
            RobotState.getInstance().getTuningTempPose().getRotation().getDegrees()
          });
    } else {
      Logger.recordOutput("RobotState/tuningTempPose", new double[] {0, 0, 0});
    }

    Logger.recordOutput(
        "Drive/EstimatedPose",
        new double[] {
          drive.getPose().getX(), drive.getPose().getY(), drive.getPose().getRotation().getDegrees()
        });

    Logger.recordOutput("Superstructure/CurrentSuperState", currentState.toString());
    Logger.recordOutput("Superstructure/DesiredSuperState", desiredState.toString());

    if (currentState == SuperState.STOPPED) handleStopped();
}

  /**
   * Sets currentState to the appropiate transition state based on desiredState
   *
   * @return The current super state
   */
  private SuperState handleStateTransitions() {
    previousState = currentState;
    var ready = ready(desiredState);
    currentState =
        switch (desiredState) {
          case L1 -> ready ? SuperState.L1 : SuperState.L1PREPARE;
          case L2 -> ready ? SuperState.L2 : SuperState.L2PREPARE;
          case L3 -> ready ? SuperState.L3 : SuperState.L3PREPARE;
          case L4 -> ready ? SuperState.L4 : SuperState.L4PREPARE;
          case INTAKE -> ready ? SuperState.INTAKE : SuperState.INTAKEPREPARE;
          case INTAKELOW -> ready ? SuperState.INTAKELOW : SuperState.INTAKELOWPREPARE;
          default -> desiredState;
        };

    return currentState;
  }

  private void applyStates()
  {
    switch (currentState) {
      case L1:
        elevator.setWantedState(Elevator.WantedState.L1);
        wrist.setWantedState(Wrist.WantedState.L1);
        break;
      case L2:
        elevator.setWantedState(Elevator.WantedState.L2);
        wrist.setWantedState(Wrist.WantedState.L2);
        break;
      case L3:
        elevator.setWantedState(Elevator.WantedState.L3);
        wrist.setWantedState(Wrist.WantedState.L3);
        break;
      case L4:
        elevator.setWantedState(Elevator.WantedState.L4);
        wrist.setWantedState(Wrist.WantedState.L4);
        break;
      case INTAKE:
        elevator.setWantedState(Elevator.WantedState.INTAKE);
        wrist.setWantedState(Wrist.WantedState.INTAKE);
        break;
      case INTAKELOW: 
        elevator.setWantedState(Elevator.WantedState.INTAKELOW);
        wrist.setWantedState(Wrist.WantedState.INTAKELOW);
        break; 
      case INTAKEPREPARE:
        elevator.setWantedState(Elevator.WantedState.INTAKE);
        wrist.setWantedState(Wrist.WantedState.INTAKE);
        break;
      case INTAKELOWPREPARE:
        elevator.setWantedState(Elevator.WantedState.INTAKELOW);
        wrist.setWantedState(Wrist.WantedState.INTAKELOW);
        break;
      case L1PREPARE:
        elevator.setWantedState(Elevator.WantedState.L1);
        wrist.setWantedState(Wrist.WantedState.L1);
        break;
      case L2PREPARE:
        elevator.setWantedState(Elevator.WantedState.L2);
        wrist.setWantedState(Wrist.WantedState.L2);
        break;
      case L3PREPARE:
        elevator.setWantedState(Elevator.WantedState.L3);
        wrist.setWantedState(Wrist.WantedState.L3);
        break;
      case L4PREPARE:
        elevator.setWantedState(Elevator.WantedState.L4);
        wrist.setWantedState(Wrist.WantedState.L4);
        break;
      case L2L3ALGAE:
        elevator.setWantedState(Elevator.WantedState.L2L3ALGAE);
        wrist.setWantedState(Wrist.WantedState.L2L3ALGAE);
        break;
      case L3L4ALGAE:
        elevator.setWantedState(Elevator.WantedState.L3L4ALGAE);
        wrist.setWantedState(Wrist.WantedState.L3L4ALGAE);
        break;
      case STOW:
      case STOPPED:
      default:
        elevator.setWantedState(Elevator.WantedState.STOW);
        wrist.setWantedState(Wrist.WantedState.STOW);
        break;
      }
  }

  private void handleStopped() {
    drive.stop();
    elevator.stop();
  }

  /** Transition check */
  private boolean ready(SuperState state) {
    return switch (state) {
        // also has to be at alignment goal to score
      case L2, L3, L4 -> elevator.atSetPoint(state) && wrist.atSetPoint(state);
        // && container.getReefAlignController().atGoal();
        // L1 is prospectively manual driving alignment
      case L1 -> elevator.atSetPoint(state) && wrist.atSetPoint(state);
      case INTAKE -> elevator.atSetPoint(state);
      case INTAKELOW -> elevator.atSetPoint(state);
      case STOW, INTAKEPREPARE, L2L3ALGAE, L3L4ALGAE -> true;
      default -> false;
    };
  }

  public BooleanSupplier doesCommandMatch(SuperState currentState) {
    return () -> Superstructure.currentState == currentState;
  }

  /** State pushers */
  public void setWantedSuperState(SuperState desiredState) {
    Superstructure.desiredState = desiredState;
  }

  public Command setWantedSuperStateCommand(SuperState desiredState) {
    return new InstantCommand(
        () -> {
          setWantedSuperState(desiredState);
        });
  }


  
}