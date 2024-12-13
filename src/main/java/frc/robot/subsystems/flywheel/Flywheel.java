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

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import java.util.spi.CurrencyNameProvider;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.drive.Drive;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  private final double subwooferShootAngle = -3.08; //-3.5  to shoot the standds
  private final double passShotAngle = -3.08;
  
  private final double subwooferShootAngleTolerance = 0.05;
  private final double limelightShootAngleTolerance = 0.05;
  private final double passShootAngleTolerance = 0.05;

  private final double subwooferRPM;
  private final double LimelightRPM;
  private final double passRPM;
  private final double idleRPM;
  private final PIDController aimPID;
  private Drive drive;

  private final double travelAngle = -3.3;

  private Debouncer currentDebouncer = new Debouncer(0.15, DebounceType.kRising);





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




  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io, Drive drive) {

    this.drive = drive;
    this.io = io;

    aimPID = new PIDController(10, 0, 1);
    subwooferRPM = getMaxOuttakeRate() * .6;
    LimelightRPM = getMaxOuttakeRate() * .75;
    idleRPM = getMaxOuttakeRate() * .4;
    passRPM = getMaxOuttakeRate() * .5;


    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0, (0.00634));
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:

        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0, 0.00634);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    io.updateShuffleboard();
    applyStates();

  }

  private void applyStates() {
    switch (currentState) {

        case MANUAL:
        break;
        case INTAKE_DOWN:
        case INTAKE_UP:
        case HOLD_FIX_PIECE:
        case REGULAR_STATE:
            idleRev();
            break;

        case SUBWOOFER_SHOT:
          subwooferShot();
          break;
        case PREPARING_SUBWOOFER_SHOT:
            prepareForSubwooferShot();
            break;

        case LIMELIGHT_SHOT:
          LimelightShot();
          break;
        case PREPARING_LIMELIGHT_SHOT:
            prepareForLimelightShot();
            break;  

        case PASS:
          pass();
          break;
        case PREPARING_PASS:
            prepareForPass();
            break;
        
        case CLIMBER_DOWN:
        case CLIMBER_UP:
        case STOPPED:
        default:
          stop();
            
    }
}


  /**
   * idle Rev and bring to travel angle when you are in regular or note fixing state
   */
  public void idleRev(){
    runVelocity(getIdleRPM());
    
    aim(aimPIDfeed(getTravelAngle()));
  }

/**
 * revs shooter, bring it to subwoofer angle
 */
  public void prepareForSubwooferShot(){
    aim(aimPIDfeed(getSubwooferAngle()));
    runVelocity(getsubwooferRPM());
  }
/**
 * revs shooter, stops angling
 */
  public void subwooferShot(){
    aim(0);
    runVelocity(getsubwooferRPM());
  }

  


/**
 * rev shooter, bring it to right angle
 */
  public void prepareForLimelightShot(){
    aim(aimPIDfeed(drive.calculateShootAngle()));
    runVelocity(getLimelightRPM());
  }

  /**
 * rev shooter, stop aiming
 */
  public void LimelightShot(){
    aim(0);
    runVelocity(getLimelightRPM());
  }

  /**
   * rev shooter, bring it to right angle
   */
  public void prepareForPass(){
    aim(aimPIDfeed(getPassShotAngle()));
    runVelocity(getPassRPM());
  }

  /**
   * rev shooter, stop aiming
   */
  public void pass(){

    aim(0);
    runVelocity(getPassRPM());
}




  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

    /**
   * Returns the current angle of the outtake.
   * 
   * @return Angle of the outtake (rad).
   */
  public double getAngle() {
    return io.getAngle();
}


/**
 * generic get rpm with state
 * @return desired rpm for the state
 */
 public double getRPMGeneric (CurrentState state)
 {
    switch (state){

      case PREPARING_SUBWOOFER_SHOT:
      case SUBWOOFER_SHOT:
        return getsubwooferRPM();
      
      case PREPARING_LIMELIGHT_SHOT:
      case LIMELIGHT_SHOT:
        return getLimelightRPM();
      
      case PREPARING_PASS:
      case PASS:
        return getPassRPM();
      default:
        return 0;
      
    }

 }

 /**
   * Returns the subwoofer RPM
   */
  public double getsubwooferRPM() {
    return subwooferRPM;
}
 /**
   * Returns the subwoofer RPM
   */
  public double getIdleRPM() {
    return idleRPM;
}

 /**
   * Returns the Limelight and Pass RPM
   * 
   *
   */
  public double getLimelightRPM() {
    return LimelightRPM;
}

/**
   * Returns the Limelight and Pass RPM
   * 
   *
   */
  public double getPassRPM() {
    return passRPM;
}

    /**
     * return Subwoofer shot angle
     * 
     */
    public double getSubwooferAngle(){
      return subwooferShootAngle;

    }

    /**
     * return Pass shot angle
     * 
     */
    public double getPassShotAngle(){
      return passShotAngle;

    }

    /**
     * return Subwoofer Shot Tolerance (Subwoofer Shot Angle +/- this amount should still work for subwoofer shot)
     * 
     */
    public double getSubwooferAngleTolerance(){
      return subwooferShootAngleTolerance;
    }

    /**
     * return Pass shoot toleraqnce (pass  Shot Angle +/- this amount should still work for pass  shot)
     * 
     */
    public double getPassAngleTolerance(){
      return passShootAngleTolerance;
    }

    /**
     * return Subwoofer Shot Tolerance (Subwoofer Shot Angle +/- this amount should still work for limelight shot)
     * 
     */
    public double getLimelightAngleTolerance(){
      return limelightShootAngleTolerance;
    }

    /**
     * return travel mode angle
     * 
     */
    public double getTravelAngle(){
      return travelAngle;
    }


    /**
     * looks if the velocity is greater than .5 of max (.4 of max is idle, so we know shot is being tried)
     * @return if the flywheel has shot.
     */
    public boolean hasNoteShot(){
      boolean hasItShot = currentDebouncer.calculate(io.getCurrentOutput() > 10.0)
        && atRPMSetpointForShotConfirm(currentState);
        return hasItShot;
    }

    /**
     * 
     * 
     * @param DesiredAngle
     * @param DesiredRPM
     * @param subwoofer
     * @return boolean: if the flywheel is close enough to desired setpoint
     */
public boolean atSetpoint(Double DesiredAngle, Double DesiredRPM, WantedSuperState wantedSuperState) {
  switch (wantedSuperState)
  {
    case PREPARING_SUBWOOFER_SHOT:
    case SUBWOOFER_SHOT:
      return MathUtil.isNear(DesiredAngle, getAngle(), getSubwooferAngleTolerance()) && MathUtil.isNear(DesiredRPM, getVelocityRPM(), 100); //100 is the rpm tolerance for subwoofer
    case PREPARING_LIMELIGHT_SHOT:
    case LIMELIGHT_SHOT:
      return MathUtil.isNear(DesiredAngle, getAngle(), getLimelightAngleTolerance()) && MathUtil.isNear(DesiredRPM, getVelocityRPM(), 100); //30 is the rpm tolerance for limelight
    case PREPARING_PASS:
    case PASS:
      return MathUtil.isNear(DesiredAngle, getAngle(), getPassAngleTolerance()) && MathUtil.isNear(DesiredRPM, getVelocityRPM(), 50); //30 is the rpm tolerance for pass
    default:
      return false;
  }

}

/**
 * checks if it's close enough to the rpm. used to check if we have shot, with measuring the current drawn of the motors.
 * @param state
 * @return whether the shooter is close enough to the RPM to confirm that it has shot.
 */
public boolean atRPMSetpointForShotConfirm(CurrentState state) {
  
    return MathUtil.isNear(getRPMGeneric(state), getVelocityRPM(), 250); //100 is the rpm tolerance for subwoofer
   
  }




    /**
     * Engages the actuator to rotate the outtake.
     * 
     * @param actuatorPower Linear actuator power [-1.0, 1.0].
     */
    public void aim (double PIDspeed){
      
        io.rotate(PIDspeed);
    }

    public double aimPIDfeed(double angle){
      if (!MathUtil.isNear(angle, getAngle(), limelightShootAngleTolerance))
        return aimPID.calculate(getAngle(), angle);
      return 0;
    }

    
  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);

    // io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec)); //with its own radian / sec stuf


    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRPM));


    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public double getMaxOuttakeRate(){
    return io.getMaxOuttakeRate();
  }

  /** State pushers */
  public void setWantedState(CurrentState wantedState) {
    this.currentState = wantedState;
}

}