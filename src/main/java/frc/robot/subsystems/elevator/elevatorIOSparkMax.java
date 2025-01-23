// // Copyright 2021-2024 FRC 6328
// // http://github.com/Mechanical-Advantage
// //
// // This program is free software; you can redistribute it and/or
// // modify it under the terms of the GNU General Public License
// // version 3 as published by the Free Software Foundation or
// // available in the root directory of this project.
// //
// // This program is distributed in the hope that it will be useful,
// // but WITHOUT ANY WARRANTY; without even the implied warranty of
// // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// // GNU General Public License for more details.

// package frc.robot.subsystems.elevator;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import java.util.function.Supplier;

// import com.ctre.phoenix6.controls.Follower;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkLowLevel.*;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import frc.robot.Constants;
// import frc.robot.util.LoggedTunableNumber;

// /**
//  * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
//  * "CANSparkFlex".
//  */
// public class elevatorIOSparkMax implements elevatorIO {

//   private static final double GEAR_RATIO = 1.0 / 5.0;
//   public static final double maxIntakeRate = 5600.0 * GEAR_RATIO; // rpm
  

//   private final SparkMax leader = new SparkMax(24, MotorType.kBrushless);
//   private final SparkMaxConfig leaderconfig = new SparkMaxConfig();

//   private final SparkMax follower = new SparkMax(45, MotorType.kBrushless);
//   private final SparkMaxConfig followerconfig = new SparkMaxConfig();


//   private static final LoggedTunableNumber maxVelocity =
//       new LoggedTunableNumber("Arm/Velocity", .1);
//   private static final LoggedTunableNumber maxAcceleration =
//       new LoggedTunableNumber("Arm/Acceleration", .1);
  

//   private final RelativeEncoder leader_encoder = leader.getEncoder();
//   private final RelativeEncoder follower_encoder = follower.getEncoder();

//   public static final Supplier<TrapezoidProfile.Constraints> maxProfileConstraints =
//       () -> new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());
    
//   ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
//   ShuffleboardLayout intakeLayout = tab.getLayout("Intake", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);
//   private final GenericEntry m_intakeRateEntry = intakeLayout.add("Intake Rate", 0 + " rpm").getEntry();
//   private final GenericEntry m_rotateAngleEntry = intakeLayout.add("Intake Angle", 0 + " rad").getEntry();
//   private final GenericEntry m_rotateAngularSpeedEntry = intakeLayout.add("Intake Angular Speed", 0 + " rad/s").getEntry();


//   public elevatorIOSparkMax() {


    
//     leaderconfig.inverted(false);
//     followerconfig.inverted(true);

//     followerconfig.follow(leader.getDeviceId(),true);

//     leaderconfig.idleMode(IdleMode.kBrake);
//     followerconfig.idleMode(IdleMode.kBrake);

//     leaderconfig.signals.absoluteEncoderPositionPeriodMs(20);
//     followerconfig.signals.absoluteEncoderPositionPeriodMs(20);

//     leaderconfig.signals.absoluteEncoderVelocityPeriodMs(20);
//     followerconfig.signals.absoluteEncoderVelocityPeriodMs(20);


//     leaderconfig.smartCurrentLimit(80, 60);
//     followerconfig.smartCurrentLimit(80, 60);

//     leader.configure(leaderconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     follower.configure(followerconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//   }

//   @Override
//   public void updateInputs(elevatorIOInputs inputs) {

//     public TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0,0));
//     public TrapezoidProfile.State trapezoidalProfileState = new TrapezoidProfile.State();

    

//     inputs.positionRads = (leader_encoder.getPosition() + follower_encoder.getPosition()) / 2.0;


//     inputs.velocityRadsPerSec = (leader_encoder.getVelocity() + follower_encoder.getVelocity()) / 2.0;

//     inputs.appliedVoltage = ((leader.getAppliedOutput() * leader.getBusVoltage()) + follower.getAppliedOutput() * follower.getBusVoltage()) / 2.0;

//     inputs.supplyCurrentAmps = (leader.getOutputCurrent() + follower.getOutputCurrent()) / 2.0;

//     inputs.torqueCurrentAmps = 0; //can't do torquecurrent for sparkamx

//     inputs.avgTempCelsius = (leader.getMotorTemperature() + follower.getMotorTemperature()) / 2.0;
//     inputs.appliedVoltsRotate = rotate_motor.getAppliedOutput() * rotate_motor.getBusVoltage();

//     inputs.currentAmps = new double[] {intake_motor.getOutputCurrent(), rotate_motor.getOutputCurrent()};

//   }

//   /**
//      * Engages the intake.
//      * 
//      * @param intakeRate The rate of intake (rpm).
//      */
//     public void intake(double intakeRate) {
//       intake_motor.setVoltage(intakeRate);

//   }

//   /**
//    * Changes the intake angle.
//    * 
//    * @param power The power desired
//    */
//   public void rotate(double power) {
//       rotate_motor.setVoltage(power);
      
//   }

//   @Override
//     public void stop(boolean interrupted) {
//         intake_motor.stopMotor();
//         rotate_motor.stopMotor();
//       }

//   /** Returns the current angle of the intake (rad). */
//   public double getAngle() {
//       return rotate_encoder.getPosition();
//   }

//   /** Resets the angle of the intake to 0. */
//   public void reset() {
//       rotate_encoder.setPosition(0);
//   }

//   /** Displays the periodically updated intake rate on the Shuffleboard */
//   public void updateShuffleboard() {
//       m_intakeRateEntry.setString(intake_encoder.getVelocity() + " rpm");
//       m_rotateAngleEntry.setString(rotate_encoder.getPosition() + " rad");
//       m_rotateAngularSpeedEntry.setString(rotate_encoder.getVelocity() + " rad/s");
      
//   }



//   /** Returns whether the intake can be activated. */
//   public boolean canIntake() {
//       return true;
//   }

//   /** Returns whether the intake can rotate. */
//   public boolean canRotate() {
//       if (m_angularSpeed > 0 && !highLimitSwitch.get()) {
//           return false;
//       }
//       if (m_angularSpeed < 0 && !lowLimitSwitch.get()) {
//           return false;
//       }
//       return true;
//   }

//   // @Override
//   // public void configurePID(double kP, double kI, double kD) {
//   //   pid.setP(kP);
//   //   pid.setI(kI);
//   //   pid.setD(kD);
//   //   // pid.setFF(0);
//   // }
// }
