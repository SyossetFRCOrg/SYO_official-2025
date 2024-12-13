package frc.robot.subsystems.vision;

import frc.robot.subsystems.drive.Drive;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.Collections;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private ArrayBlockingQueue<Double> m_limelight_x_position, m_limelight_y_position;
    private NetworkTableEntry m_angleX;
    private Pose2d visionPose2d;
    private Pose2d previousVisionPose2d = new Pose2d();
    private Drive drive;
    NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");

    private double[] m_limelightodometry = networkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);


    // private final GenericEntry m_drivetrainAngleChangeEntry;
    private final GenericEntry m_robotPose; 

    public Vision(Drive drive) {
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_limelightodometry = networkTable.getEntry("botpose").getDoubleArray(new double[6]);

        m_limelight_x_position = new ArrayBlockingQueue<>(5, true, Collections.nCopies(3, m_limelightodometry[0]));
        m_limelight_y_position = new ArrayBlockingQueue<>(5, true, Collections.nCopies(3, m_limelightodometry[1]));
       
        m_angleX = networkTable.getEntry("tx");

        ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
        ShuffleboardLayout limelightLayout = tab.getLayout("Limelight", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0);
        // m_drivetrainAngleChangeEntry = limelightLayout.add("Desired Drivetrain Angle Change", getDrivetrainAngleChange() + " rad").getEntry();
        m_robotPose = limelightLayout.add("Limelight Robot Pose", "(" + m_limelightodometry[0] + ", " + m_limelightodometry[1] + ")").getEntry(); 
    }

    @Override
    public void periodic() {



        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        

        visionPose2d = mt2.pose;
        
        

        
        // m_drivetrainAngleChangeEntry.setString(getDrivetrainAngleChange() + " rad");
        m_robotPose.setString("(" + m_limelightodometry[0] + ", " + m_limelightodometry[1] + ")");
        
        
        
        if (!previousVisionPose2d.equals(visionPose2d)){
        
            double timeStamp = Timer.getFPGATimestamp();
            
            drive.addVisionMeasurement(visionPose2d , timeStamp);
            
            previousVisionPose2d = visionPose2d;
        }

    }

    

   

    // /**
    //  * Returns the change in drivetrain angle necessary for shooting based on limelight input.
    //  * 
    //  * @return Change in drivetrain angle (rad).
    //  */
    // public double getDrivetrainAngleChange() {
    //     return -Math.toRadians(m_angleX.getDouble(0));
    // }
}