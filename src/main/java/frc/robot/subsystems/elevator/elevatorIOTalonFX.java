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

import static frc.robot.util.PhoenixUtil.*;

import java.util.Queue;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class elevatorIOTalonFX implements elevatorIO {

  private static final double GEAR_RATIO = 1.0 / 5.0;
  public static final double maxIntakeRate = 5600.0 * GEAR_RATIO; // rpm
  

  final MotionMagicVoltage elevatorRequest = new MotionMagicVoltage(0);

  private final TalonFX motor1;

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/Gains/kP", 0);
    // private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/Gains/kI", 0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/Gains/kD", 0);
    private static final LoggedTunableNumber kS =
        new LoggedTunableNumber("Arm/Gains/kS", 0);
    private static final LoggedTunableNumber kV =
        new LoggedTunableNumber("Arm/Gains/kV", 0);
    private static final LoggedTunableNumber kA =
        new LoggedTunableNumber("Arm/Gains/kA", 0);
    private static final LoggedTunableNumber kG =
        new LoggedTunableNumber("Arm/Gains/kG", 0);


    private static final LoggedTunableNumber motionMagicVelocity =
        new LoggedTunableNumber("Arm/maxVelocity", .1);
    private static final LoggedTunableNumber motionMagicAcceleration =
        new LoggedTunableNumber("Arm/maxAcceleration", .1);
    private static final LoggedTunableNumber motionMagicJerk =
        new LoggedTunableNumber("Arm/maxJerk", .1);
    

    private final StatusSignal<Angle> elevatorPosition;
    private final StatusSignal<AngularVelocity> elevatorVelocity;
    private final StatusSignal<Voltage> elevatorAppliedVolts;
    private final StatusSignal<Current> elevatorCurrent;
    private final StatusSignal<Current> elevatorTorqueCurrent;
    private final StatusSignal<Temperature> tempCelsius;



    private final Debouncer elevatorConnectedDebounce = new Debouncer(0.5);

      
        
//   ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
//   ShuffleboardLayout intakeLayout = tab.getLayout("Intake", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);
//   private final GenericEntry m_intakeRateEntry = intakeLayout.add("Intake Rate", 0 + " rpm").getEntry();
//   private final GenericEntry m_rotateAngleEntry = intakeLayout.add("Intake Angle", 0 + " rad").getEntry();
//   private final GenericEntry m_rotateAngularSpeedEntry = intakeLayout.add("Intake Angular Speed", 0 + " rad/s").getEntry();


  public elevatorIOTalonFX() {


    motor1 = new TalonFX(16, "rio");

    var motor1config = new TalonFXConfiguration();

    motor1config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor1config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    motor1config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    motor1config.Slot0.kA = kA.get();
    motor1config.Slot0.kD = kD.get();
    motor1config.Slot0.kG = kG.get();
    motor1config.Slot0.kP = kP.get();
    motor1config.Slot0.kS = kS.get();
    motor1config.Slot0.kV = kV.get();

    motor1config.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration.get();
    motor1config.MotionMagic.MotionMagicCruiseVelocity = motionMagicVelocity.get();
    motor1config.MotionMagic.MotionMagicJerk = motionMagicJerk.get();
    

    motor1config.Feedback.SensorToMechanismRatio = 1.0;
    // motor1config.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    // motor1config.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    motor1config.CurrentLimits.StatorCurrentLimit = 80;
    motor1config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor1config.CurrentLimits.SupplyCurrentLimit = 60;
    motor1config.CurrentLimits.SupplyCurrentLimitEnable = true;


    motor1config.MotorOutput.Inverted =
        false //fix this, test this.  Positive should be upward
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
            
    tryUntilOk(5, () -> motor1.getConfigurator().apply(motor1config, 0.25));
    tryUntilOk(5, () -> motor1.setPosition(0.0, 0.25));

    elevatorPosition = motor1.getPosition();
    elevatorVelocity = motor1.getVelocity();
    elevatorAppliedVolts = motor1.getMotorVoltage();
    elevatorCurrent = motor1.getSupplyCurrent();
    elevatorTorqueCurrent = motor1.getTorqueCurrent();
    tempCelsius = motor1.getDeviceTemp();


    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, 
        elevatorPosition,
        elevatorVelocity,
        elevatorAppliedVolts,
        elevatorCurrent,
        elevatorTorqueCurrent,
        tempCelsius);
    ParentDevice.optimizeBusUtilizationForAll(motor1);

  }

  @Override
  public void updateInputs(elevatorIOInputs inputs) {

    var motor1Status =
        BaseStatusSignal.refreshAll(elevatorPosition,
        elevatorVelocity,
        elevatorAppliedVolts,
        elevatorCurrent,
        elevatorTorqueCurrent,
        tempCelsius);
    

    inputs.motorConnected = elevatorConnectedDebounce.calculate(motor1Status.isOK());

    inputs.positionRads = Units.rotationsToRadians(elevatorPosition.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(elevatorVelocity.getValueAsDouble());
    inputs.appliedVoltage = elevatorAppliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = elevatorCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = elevatorTorqueCurrent.getValueAsDouble();
    inputs.avgTempCelsius = tempCelsius.getValueAsDouble();

  }

  @Override
    public void stop() {
        motor1.stopMotor();
      }

  /** Resets the angle of the intake to 0. */
  public void set(double positionRads) {
      motor1.setPosition(Units.radiansToRotations(positionRads));
  }


  /** Run elevator to position - Motion Magic*/
  public void movetoHeight(double posRads) {
    motor1.setControl(elevatorRequest.withPosition(Units.radiansToRotations(posRads)));

  }




//   /** Displays the periodically updated intake rate on the Shuffleboard */
//   public void updateShuffleboard() {
//       m_intakeRateEntry.setString(intake_encoder.getVelocity() + " rpm");
//       m_rotateAngleEntry.setString(rotate_encoder.getPosition() + " rad");
//       m_rotateAngularSpeedEntry.setString(rotate_encoder.getVelocity() + " rad/s");
      
//   }

  // @Override
  // public void configurePID(double kP, double kI, double kD) {
  //   pid.setP(kP);
  //   pid.setI(kI);
  //   pid.setD(kD);
  //   // pid.setFF(0);
  // }


}
