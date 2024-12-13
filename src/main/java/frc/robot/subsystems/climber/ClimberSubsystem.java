// package frc.robot.subsystems.climber;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class ClimberSubsystem extends SubsystemBase {
//     private final CANSparkMax m_climberMotorLeft;
//     private final CANSparkMax m_climberMotorRight;

//     RelativeEncoder left_encoder;
//     RelativeEncoder right_encoder;
//     private double m_climberPowerLeft;
//     private double m_climberPowerRight;

//     private final GenericEntry m_climberPowerLeftEntry;

//     private final GenericEntry m_climberPowerRightEntry;

//     private final GenericEntry left_Climber_Height;
//     private final GenericEntry right_Climber_Height;

//     public ClimberSubsystem() {
//         m_climberMotorLeft = new CANSparkMax(10, MotorType.kBrushless);
//         m_climberMotorRight = new CANSparkMax(14, MotorType.kBrushless);

//         left_encoder = m_climberMotorLeft.getEncoder();
//         right_encoder = m_climberMotorRight.getEncoder();

//         left_encoder.setPosition(0);
//         right_encoder.setPosition(0);

//         m_climberMotorLeft.setIdleMode(IdleMode.kBrake);
//         m_climberMotorRight.setIdleMode(IdleMode.kBrake);

//         m_climberMotorLeft.setInverted(false);
//         m_climberMotorRight.setInverted(true);

//         ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
//         ShuffleboardLayout climberLayout = tab.getLayout("Climber", BuiltInLayouts.kList).withSize(2, 2).withPosition(4, 0);
//         m_climberPowerLeftEntry = climberLayout.add("Left Climber Power", m_climberPowerLeft).getEntry();
//         m_climberPowerRightEntry = climberLayout.add("Right Climber Power", m_climberPowerRight).getEntry();
//         left_Climber_Height = climberLayout.add("Left Climber Height", left_encoder.getPosition()).getEntry();
//         right_Climber_Height = climberLayout.add("Right Climber Height", right_encoder.getPosition()).getEntry();


//     }

//     /**
//      * Engages the climber.
//      * 
//      * @param climberPowerLeft The power of the left climber [-1, 1].
//      * @param climberPowerRight The power of the right climber [-1, 1].
//      */
//     public void climb(double climberPowerLeft, double climberPowerRight) {
//         m_climberPowerLeft = climberPowerLeft;
//         m_climberPowerRight = climberPowerRight;
//     }

//     public void stop(){
//         m_climberMotorRight.stopMotor();
//         m_climberMotorLeft.stopMotor();
//     }




//     @Override
//     public void periodic() {
//         // if (left_encoder.getPosition() < 4 && m_climberPowerLeft > 0)
//             m_climberMotorLeft.set(m_climberPowerLeft);
//         // else
//         //     m_climberMotorLeft.stopMotor();
        
//         // if (right_encoder.getPosition() < 4 && m_climberPowerRight > 0)
//             m_climberMotorRight.set(m_climberPowerRight);
//         // else
//         //     m_climberMotorRight.stopMotor();

//         m_climberPowerLeftEntry.setDouble(m_climberPowerLeft);
//         m_climberPowerRightEntry.setDouble(m_climberPowerRight);
//         left_Climber_Height.setDouble(left_encoder.getPosition());
//         right_Climber_Height.setDouble(right_encoder.getPosition());
//     }
// }
