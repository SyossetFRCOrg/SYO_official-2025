// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.elevator.elevator.elevatorState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.swerve.ModuleLimits;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class elevator extends SubsystemBase {
  private final elevatorIO io;
  private final elevatorIOInputsAutoLogged inputs = new elevatorIOInputsAutoLogged();
  

  
  @RequiredArgsConstructor
  public static enum elevatorState {


    // ALL OF THESE VALUES DEPEND ON THE MOTOR BEING USED FOR POWERING THE ELEVATOR
    // IT CAN CHANGE IF THERE IS A DIFFERENT MOTOR (Hence, different gear ratio/encoder sensitivity and whatnot)
    // don't mix and match motors
    // 


    //initially, this is going to be tuned for NEOS.
    STOW(new LoggedTunableNumber("elevator/StowPosition", 0)),
    INTAKE(new LoggedTunableNumber("elevator/IntakePosition", 3)),

    L1PREPARE(new LoggedTunableNumber("elevator/L1Position", 4)),
    L2PREPARE(new LoggedTunableNumber("elevator/L2Position", 5)),
    L3PREPARE(new LoggedTunableNumber("elevator/L3Position", 6)),
    L4PREPARE(new LoggedTunableNumber("elevator/L4Position", 7)),

    L1(new LoggedTunableNumber("elevator/L1Position", 5)),
    L2(new LoggedTunableNumber("elevator/L2Position", 6)),
    L3(new LoggedTunableNumber("elevator/L3Position", 7)),
    L4(new LoggedTunableNumber("elevator/L4Position", 8));
    
    // CUSTOM(new LoggedTunableNumber("Elevator/CustomSetpoint", 20.0));

    private final DoubleSupplier desiredheightSupplier;

    private double get() {
      return desiredheightSupplier.getAsDouble();
    }
  }


  @AutoLogOutput @Getter @Setter private elevatorState state = elevatorState.STOW;

    
  private final SimpleMotorFeedforward intakeFeedforward;
  private final SimpleMotorFeedforward rotateFeedforward;
  private final PIDController pid;




  /** Creates a new Flywheel. */
  public elevator(elevatorIO io) {
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
    Logger.processInputs("elevator", inputs);
    io.updateShuffleboard();
    
    if (inputs.motorType.equals("Sparkmax"))
    {
      io.periodic();
    }

    applyStates();
  }
  private void applyStates() {
    switch (state) {

       
        case INTAKE:
          io.movetoHeight(state.get());
          break;
        case L1:
          io.movetoHeight(state.get());
          break;
        case L2:
          io.movetoHeight(state.get());
          break;
        case L3:
          io.movetoHeight(state.get());
          break;
        case L4:
          io.movetoHeight(state.get());
          break;
        case STOW:
        default:
          io.movetoHeight(state.get());
          break;

      }
}

  // /** Returns a command to run a quasistatic test in the specified direction. */
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return sysId.quasistatic(direction);
  // }

  // /** Returns a command to run a dynamic test in the specified direction. */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return sysId.dynamic(direction);
  // }

  /**
     * 
     * @return boolean: if the intake is close enough to desired setpoint
     */
  public boolean atSetPoint() {
    return MathUtil.isNear(state.get(), getHeight(), 0.1);
  }

  /** Returns the current angle of the intake (rad). */
  public double getHeight() {
      return inputs.positionRads;
  }

  public boolean elevatorup(){
    return getHeight() >= elevatorState.L2.get();
  }

  public ModuleLimits getModuleLimits() {
            return elevatorup() && !DriverStation.isAutonomousEnabled()
            ? TunerConstants.moduleLimitsElevatorUp
            : TunerConstants.moduleLimitsFree;
    }

  /** Resets the angle of the intake to positionRads. */
  public void setHeight(double positionRads) {
      io.setHeight(positionRads);
  }
  
  public void stop(){
    io.stop();
  }

  public void setWantedState(elevatorState wantedState) {
    this.state = wantedState;
}



}