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

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.flywheel.Flywheel.WantedState;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  

  // private final SysIdRoutine sysId;

  private final double intakingAngle = -3.35;
  private final double shootingAngle = 0;
  
  public static enum WantedState {
    HOLD_FIX_PIECE,
    REGULAR_STATE,
    PREPARING_SUBWOOFER_SHOT,
    SUBWOOFER_SHOT,
    PREPARING_LIMELIGHT_SHOT,
    LIMELIGHT_SHOT,
    PREPARING_PASS,
    PASS,
    MANUAL,

    INTAKE_DOWN,
    INTAKE_UP,
    CLIMBER_UP,
    CLIMBER_DOWN,
    STOPPED,
  }


  public static enum CurrentState {
      HOLD_FIX_PIECE,
      REGULAR_STATE,
      PREPARING_SUBWOOFER_SHOT,
      SUBWOOFER_SHOT,
      PREPARING_LIMELIGHT_SHOT,
      LIMELIGHT_SHOT,
      PREPARING_PASS,
      PASS,
      MANUAL,

      INTAKE_DOWN,
      INTAKE_UP,
      CLIMBER_UP,
      CLIMBER_DOWN,
      STOPPED,
  }


  public static CurrentState currentState = CurrentState.STOPPED;

    
  private final SimpleMotorFeedforward intakeFeedforward;
  private final SimpleMotorFeedforward rotateFeedforward;
  private final PIDController pid;




  /** Creates a new Flywheel. */
  public Intake(IntakeIO io) {
    intakeFeedforward = new SimpleMotorFeedforward(0, 0.01057);
    rotateFeedforward = new SimpleMotorFeedforward(0, 3);
    pid = new PIDController(3,0,0.05);

    this.io = io;


    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        

        io.configurePID(3, 0, 0.05); // rotate pid
        break;
      case SIM:

        
        io.configurePID(0.5, 0.0, 0.0); //rotate pid
        break;
      default:
        
        break;
    }

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
    Logger.processInputs("intake", inputs);
    io.updateShuffleboard();
    applyStates();
  }
  private void applyStates() {
    switch (currentState) {

        case MANUAL:
          break;

        case INTAKE_DOWN:
          intakeDown();
        break;
        
        case SUBWOOFER_SHOT:        
        case LIMELIGHT_SHOT:      
        case PASS:
          shoot();
          break;
        case INTAKE_UP:
               

        case PREPARING_LIMELIGHT_SHOT:
        case PREPARING_SUBWOOFER_SHOT:
        case PREPARING_PASS:
        case HOLD_FIX_PIECE:
        case REGULAR_STATE:
            
        case CLIMBER_DOWN:
        case CLIMBER_UP:
        case STOPPED:
        default:
          makeSureIntakeUp();
          break;
            
    }
}

  /**
   * does it
   */
  public void makeSureIntakeUp(){
    intake(() -> intakeFeedforward.calculate(0));           

    if (!atShootPoint())
    {
        rotate(() -> rotateFeedforward.calculate(pid.calculate(getAngle(), 0))); 
    }
  }
  
  /**
   * does it
   */
  public void intakeDown(){
    rotate(() -> rotateFeedforward.calculate(pid.calculate(getAngle(), intakingAngle) )); //to intakeangle (variable) usually, switch back 
    intake(() -> intakeFeedforward.calculate(-500.0));
  }

  // /**
  //  * does it
  //  */
  // public void intakeUp(){
  //   rotate(() -> rotateFeedforward.calculate(pid.calculate(getAngle(), 0))); 
  //   intake(() -> intakeFeedforward.calculate(0));
  // }

  /**
   * does it
   */
  public void shoot(){
    intake(() -> intakeFeedforward.calculate(700.0));
  }


  public SimpleMotorFeedforward getIntakeFeedForward(){
    return intakeFeedforward;
  }
  public SimpleMotorFeedforward getRotateFeedForward(){
    return rotateFeedforward;
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
     * Engages the intake.
     * 
     * @param intakeRate The rate of intake (rpm).
     */
    public void intake(DoubleSupplier intakeRate) {
      io.intake((intakeRate.getAsDouble()));
  }

  /**
   * Changes the intake angle.
   * 
   * @param angle The desired angle (rad) (shootposition angle = 0 rad)
   */
  public void rotate(DoubleSupplier angle) {
      io.rotate(angle.getAsDouble());
  }

  

  /**
   * brings intake to shooting positon
   * 
   * 
   */
  public void rotateToShootPos() {
      io.rotate(shootingAngle);
  }
  /**
   * brings intake to intake positon
   * 
   * 
   */
  public void rotateToIntakePos() {
      io.rotate(3);
  }

  



  /**
     * 
     * @return boolean: if the intake is close enough to desired setpoint
     */
  public boolean atShootPoint() {
    
    return MathUtil.isNear(shootingAngle, getAngle(), 0.1);
  }

  /** Returns the current angle of the intake (rad). */
  public double getAngle() {
      return io.getAngle();
  }

  /** Resets the angle of the intake to 0. */
  public void reset() {
      io.reset();
  }
  
    /** Returns whether the intake can be activated. */
    public boolean canIntake() {
      return io.canIntake();
  }

  /** Returns whether the intake can rotate. */
  public boolean canRotate() {
      return io.canRotate();
  }

  public void stop(){
    io.stop(true);
  }

  public void setWantedState(CurrentState wantedState) {
    this.currentState = wantedState;
}



}