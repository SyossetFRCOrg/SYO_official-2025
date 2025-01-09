package frc.robot;

import java.util.List;

import com.techhounds.houndutil.houndlib.leds.BaseLEDSection;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.LEDs.LEDState;

public class Constants {
    public static final double LOOP_TIME = 0.020; // 20ms

    public static final class Drivetrain {

        public static final int LEFT_PRIMARY_MOTOR_ID = 3;
        public static final int LEFT_SECONDARY_MOTOR_ID = 4;
        public static final int RIGHT_PRIMARY_MOTOR_ID = 1;
        public static final int RIGHT_SECONDARY_MOTOR_ID = 2;

        public static final boolean LEFT_DRIVE_MOTORS_INVERTED = false;
        public static final boolean RIGHT_DRIVE_MOTORS_INVERTED = true;

        /** Distance between left and right wheels. */
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(22);

        public static final double GEARING = 8.45;
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3) / 0.9788265228;
        public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / GEARING;

        public static final int CURRENT_LIMIT = 60; // TODO
        public static final DCMotor GEARBOX_REPR = DCMotor.getNEO(2);
        public static final double MOI = 2;
        public static final double MASS_KG = 20; // TODO

        public static final double VELOCITY_kP = RobotBase.isReal() ? 1.5 : 1.4078;
        public static final double VELOCITY_kI = 0.0;
        public static final double VELOCITY_kD = 0.0;
        public static final double kS = RobotBase.isReal() ? 0.29971 : 0.057644;
        public static final double kV_LINEAR = RobotBase.isReal() ? 2.4 : 2.159;
        public static final double kA_LINEAR = RobotBase.isReal() ? 0.56555 : 0.24174;
        public static final double kV_ANGULAR = RobotBase.isReal() ? 1.2427 : 2.1403;
        public static final double kA_ANGULAR = RobotBase.isReal() ? 0.0859 : 0.42056;

        public static final double POSITION_kP = 1.5;
        public static final double POSITION_kI = 0.0;
        public static final double POSITION_kD = 0.0;
        public static final double ROTATION_kP = 2.0;
        public static final double ROTATION_kI = 0.0;
        public static final double ROTATION_kD = 0.0;

        public static final double RAMSETE_B = 3; // TODO
        public static final double RAMSETE_ZETA = 0.7; // TODO

        public static final double MAX_DRIVING_VELOCITY_METERS_PER_SECOND = 5;
        public static final double MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 8; // TODO
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 20;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 40;

        public static final double PATH_FOLLOWING_TRANSLATION_kP = 8.0; // TODO
        public static final double PATH_FOLLOWING_ROTATION_kP = 8.0; // TODO

        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED);
        public static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH_METERS);
    }

    public static final class Elevator {
        public static enum ElevatorPosition {
            BOTTOM(0.0698),
            INTAKE_PREP(0.55),
            INTAKE(0.355),
            ALGAE_L2(0.884),
            ALGAE_L3(1.234),

            L1(0.323),
            L2(0.31),
            L3(0.70),
            L4(1.27),
            TOP(1.57);

            public final double value;

            private ElevatorPosition(double value) {
                this.value = value;
            }
        }

        public static final double MOTION_LIMIT = 0.3;

        public static final double SCORING_MOVEMENT = -0.25;

        public static final int MOTOR_ID = 5;
        public static final boolean MOTOR_INVERTED = false;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
        public static final double GEARING = 5.0;
        public static final double MASS_KG = Units.lbsToKilograms(20);
        public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.32) / 2.0; // TODO
        public static final double DRUM_CIRCUMFERENCE = 2.0 * Math.PI * DRUM_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = DRUM_CIRCUMFERENCE / GEARING;

        public static final double MIN_HEIGHT_METERS = 0.005; // TODO
        public static final double MAX_HEIGHT_METERS = 1.57; // TODO

        public static final int CURRENT_LIMIT = 60;

        public static final double kP = 50; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 5; // TODO
        public static final double kS = 0.095388; // TODO
        public static final double kG = 0.54402; // TODO
        public static final double kV = 7.43; // TODO
        public static final double kA = 1.0; // TODO
        public static final double TOLERANCE = 0.02;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 1.3; // TODO
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; // TODO
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }

    public static final class Arm {
        public static enum ArmPosition {
            BOTTOM(-Math.PI / 2.0 + Units.degreesToRadians(5)),
            HORIZONTAL(0),
            L1(0),
            L2(Units.degreesToRadians(55)), // reef angle
            L3(Units.degreesToRadians(55)),
            L4(1.033),
            TOP(Math.PI / 2.0);

            public final double value;

            private ArmPosition(double value) {
                this.value = value;
            }
        }

        public static final double MOTION_LIMIT = -0.7;
        public static final double SCORING_MOVEMENT = -0.8;

        public static final int MOTOR_ID = 12;
        public static final boolean MOTOR_INVERTED = true;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
        public static final double GEARING = 40.0; // TODO
        public static final double MASS_KG = Units.lbsToKilograms(10); // TODO
        public static final double COM_DISTANCE_METERS = Units.inchesToMeters(6); // TODO
        public static final double MOI = SingleJointedArmSim.estimateMOI(COM_DISTANCE_METERS, MASS_KG);
        public static final double ENCODER_ROTATIONS_TO_METERS = 2 * Math.PI / GEARING;

        public static final double MIN_ANGLE_RADIANS = -Math.PI / 2.0;
        public static final double MAX_ANGLE_RADIANS = Math.PI / 2.0;

        public static final int CURRENT_LIMIT = 50;

        public static final double kP = 10; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO
        public static final double kS = 0.017964; // TODO
        public static final double kG = 0.321192; // TODO
        public static final double kV = 0.876084;// TODO
        public static final double kA = 0.206676;// TODO
        public static final double TOLERANCE = 0.02;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 8; // TODO
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 4; // TODO
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }

    public static final class Intake {
        public static final int MOTOR_ID = 13;
        public static final boolean MOTOR_INVERTED = true;
        public static final int CURRENT_LIMIT = 60;
    }

    public static final class Climber {
        public static final int MOTOR_ID = 6;
        public static final boolean MOTOR_INVERTED = false;
        public static final int CURRENT_LIMIT = 60;

        public static final double MIN_POSITION_METERS = 0.0;
        public static final double MAX_POSITION_METERS = 1.0; // TODO

        public static final double GEARING = 64.0;
        public static final double MASS_KG = Units.lbsToKilograms(80); // robot weight
        public static final double SPOOL_RADIUS_METERS = Units.inchesToMeters(0.5);
        public static final double SPOOL_CIRCUMFERENCE = 2.0 * Math.PI * SPOOL_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = SPOOL_CIRCUMFERENCE * GEARING;

    }

    public static final class LEDs {
        public static enum LEDSection implements BaseLEDSection {
            ELEVATOR_LEFT(5, 59, false),
            ELEVATOR_TOP(60, 89, true),
            ELEVATOR_TOP_LEFT(60, 74, false),
            ELEVATOR_TOP_RIGHT(75, 89, true),
            ELEVATOR_RIGHT(90, 146, true),
            MIDDLE(147, 167, false),
            HOPPER_RIGHT(166, 198, true),
            HOPPER_RIGHT_FULL(166, 218, true),
            HOPPER_TOP(199, 238), // center 218
            HOPPER_LEFT(239, 273),
            HOPPER_LEFT_FULL(219, 273),
            HOPPER_ARCH(274, 298),
            HOPPER_ARCH_LEFT(274, 289),
            HOPPER_ARCH_RIGHT(290, 305, true),
            ALL(0, 305, true);

            private final int startIdx;
            private final int endIdx;
            private final boolean inverted;

            private LEDSection(int startIdx, int endIdx, boolean inverted) {
                this.startIdx = startIdx;
                this.endIdx = endIdx;
                this.inverted = inverted;
            }

            private LEDSection(int startIdx, int endIdx) {
                this(startIdx, endIdx, false);
            }

            @Override
            public int start() {
                return startIdx;
            }

            @Override
            public int end() {
                return endIdx;
            }

            @Override
            public boolean inverted() {
                return inverted;
            }

            @Override
            public int length() {
                return endIdx - startIdx + 1;
            }
        }

        public static final int PORT = 0;
        public static final int LENGTH = 333;

        public static final List<LEDState> DEFAULT_STATES = List.of();
    }

    public static final class Controls {
        public static final double JOYSTICK_INPUT_RATE_LIMIT = 15.0;
        public static final double JOYSTICK_INPUT_DEADBAND = 0.1;
        public static final double JOYSTICK_CURVE_EXP = 2;
        public static final double JOYSTICK_ROT_CURVE_EXP = 3;
        public static final double JOYSTICK_ROT_LIMIT = 0.8;
    }
}
