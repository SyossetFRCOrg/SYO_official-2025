package frc.robot;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AllianceFlipUtil;
// import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import frc.robot.util.GeomUtil;
import frc.robot.util.swerve.ModuleLimits;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
// import org.littletonrobotics.frc2024.subsystems.drive.DriveConstants;
// import org.littletonrobotics.frc2024.subsystems.superstructure.arm.ArmConstants;
// import org.littletonrobotics.frc2024.util.AllianceFlipUtil;
// import org.littletonrobotics.frc2024.util.GeomUtil;
// import org.littletonrobotics.frc2024.util.LoggedTunableNumber;
// import org.littletonrobotics.frc2024.util.NoteVisualizer;
// import org.littletonrobotics.frc2024.util.swerve.ModuleLimits;
import org.littletonrobotics.junction.AutoLogOutput;

@ExtensionMethod({GeomUtil.class})
public class RobotState {

  private RobotState() {}

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  @AutoLogOutput(key = "RobotState/RobotPose")
  @Getter
  @Setter
  private volatile Pose2d RobotPose = new Pose2d();

  @AutoLogOutput(key = "RobotState/aboveL1")
  @Getter
  @Setter
  private volatile boolean aboveL1 = false;

  @AutoLogOutput(key = "RobotState/elevatorPosition")
  @Getter
  @Setter
  private volatile int elevatorPosition = 0;

  @AutoLogOutput(key = "RobotState/addingVision")
  @Getter
  @Setter
  private volatile boolean addingVision = true;

  @AutoLogOutput(key = "RobotState/wristCanMove")
  @Getter
  @Setter
  private volatile boolean wristCanMove = false;

  @AutoLogOutput(key = "RobotState/reefAutoAligning")
  @Getter
  @Setter
  private volatile boolean reefAutoAligning = false;

  @AutoLogOutput(key = "RobotState/reefAutoAiming")
  @Getter
  @Setter
  private volatile boolean reefAutoAiming = false;

  @AutoLogOutput(key = "RobotState/intakeAutoAiming")
  @Getter
  @Setter
  private volatile boolean intakeAutoAiming = true;

  @AutoLogOutput(key = "RobotState/tuningTempPose")
  @Getter
  @Setter
  private volatile Pose2d tuningTempPose = null;

  private Pose2d[] coralStationPositions = {
    new Pose2d(1.1344856023788452, 7.127560615539551, Rotation2d.fromRadians(2.200791626297564)),
    new Pose2d(1.0559332370758057, 0.9723778963088989, Rotation2d.fromRadians(-2.202456197984347)),
    new Pose2d(16.345178604125977, 0.8527500629425049, Rotation2d.fromRadians(-0.9320003578681158)),
    new Pose2d(16.37734603881836, 7.141775608062744, Rotation2d.fromRadians(0.9389806255220727)),
  };

  // in order, blue A-L and then red A-L
  public static final int[] tagList = {18, 17, 22, 21, 20, 19, 7, 8, 9, 10, 11, 6};

  // for practice fields like Arumdaun
  private Pose2d[][] reefscoringPositionsPractice = // not finalized or tuned.
      {
    {
      new Pose2d(3.035805043411255, 4.182767868041992, Rotation2d.fromDegrees(0)),
      new Pose2d(3.035805043411255, 3.8617191314697266, Rotation2d.fromDegrees(0))
    },
    {
      new Pose2d(3.6188, 2.84801, Rotation2d.fromDegrees(60)),
      new Pose2d(3.8880816027179366, 2.6979360979593423, Rotation2d.fromDegrees(55.55251942824823)),
    },
    {
      new Pose2d(4.959644001960754395, 2.68101, Rotation2d.fromDegrees(120)),
      new Pose2d(5.293663674429707, 2.8533856345831374, Rotation2d.fromDegrees(119.27960984178554)),
    },
    {
      new Pose2d(
          5.814974784851074 + Units.inchesToMeters(5),
          3.8649332523345947,
          Rotation2d.fromDegrees(180)),
      new Pose2d(
          5.814974784851074 + Units.inchesToMeters(5),
          4.177645683288574,
          Rotation2d.fromDegrees(180)),
    },
    {
      new Pose2d(5.368289222717285, 5.235887756347656, Rotation2d.fromDegrees(-120)),
      new Pose2d(5.111470928192139, 5.392832508087158, Rotation2d.fromDegrees(-120)),
    },
    {
      new Pose2d(3.8932504449234008789, 5.3975392832508087158, Rotation2d.fromDegrees(-60)),
      new Pose2d(3.602333632564544678, 5.231181619834899902, Rotation2d.fromDegrees(-60)),
    },

    // red alliance, also in order of A, B, C, etc
    {
      new Pose2d(14.539, 3.900, Rotation2d.fromDegrees(-178.379)),
      new Pose2d(
          14.44165855591995367, 4.2523386002392076, Rotation2d.fromDegrees(-180.17210675287777))
    },
    {
      new Pose2d(13.991158714294434, 5.256573905944824, Rotation2d.fromDegrees(-120)),
      new Pose2d(13.708047142028809, 5.413857688903809, Rotation2d.fromDegrees(-120)),
    },
    {
      new Pose2d(
          12.448975945229831, 5.391850951541687, Rotation2d.fromDegrees(-57.931551216345206)),
      new Pose2d(12.103763639987946, 5.297896918328212, Rotation2d.fromDegrees(-58.27866028322585)),
    },
    {
      new Pose2d(11.588984260559082, 4.186556816101074, Rotation2d.fromRadians(0)),
      new Pose2d(11.588498611450195, 3.8510172367095947, Rotation2d.fromRadians(0)),
    },
    {
      new Pose2d(12.20763469696045, 2.7914857959747314, Rotation2d.fromDegrees(60)),
      new Pose2d(12.480260620117188, 2.602744827270508, Rotation2d.fromDegrees(60)),
    },
    {
      AllianceFlipUtil.apply(
          new Pose2d(3.8932504449234008789, 5.3975392832508087158, Rotation2d.fromDegrees(-60))),
      AllianceFlipUtil.apply(
          new Pose2d(3.602333632564544678, 5.231181619834899902, Rotation2d.fromDegrees(-60))),
    },
  };

  // for official fields // not finalized or tuned.
  // DON'T use alliancefliputil to flip to get corresponding red scoring pose2ds THEY ARE ALL HERE
  // AND WILL BE TUNED FOR EACH FIELD
  // blue positions, in the order of A, B, C, etc. Using tuningTempPose to tune on practice day
  private Pose2d[][] reefscoringPositionsComp = {
    {
      new Pose2d(3.035805043411255, 4.182767868041992, Rotation2d.fromDegrees(0)),
      new Pose2d(3.035805043411255, 3.8617191314697266, Rotation2d.fromDegrees(0))
    },
    {
      new Pose2d(3.6188, 2.84801, Rotation2d.fromDegrees(60)),
      new Pose2d(3.895, 2.68101, Rotation2d.fromDegrees(60)),
    },
    {
      new Pose2d(4.959644001960754395, 2.68101, Rotation2d.fromDegrees(120)),
      new Pose2d(5.3597538566589355, 2.84801, Rotation2d.fromDegrees(120)),
    },
    {
      new Pose2d(
          5.814974784851074 + Units.inchesToMeters(5),
          3.8649332523345947,
          Rotation2d.fromDegrees(180)),
      new Pose2d(
          5.814974784851074 + Units.inchesToMeters(5),
          4.177645683288574,
          Rotation2d.fromDegrees(180)),
    },
    {
      new Pose2d(5.368289222717285, 5.235887756347656, Rotation2d.fromDegrees(-120)),
      new Pose2d(5.111470928192139, 5.392832508087158, Rotation2d.fromDegrees(-120)),
    },
    {
      new Pose2d(3.8932504449234008789, 5.3975392832508087158, Rotation2d.fromDegrees(-60)),
      new Pose2d(3.602333632564544678, 5.231181619834899902, Rotation2d.fromDegrees(-60)),
    },

    // red alliance, also in order of A, B, C, etc
    {
      new Pose2d(14.539, 3.900, Rotation2d.fromDegrees(-178.379)),
      new Pose2d(
          14.44165855591995367, 4.2523386002392076, Rotation2d.fromDegrees(-180.17210675287777))
    },
    {
      new Pose2d(13.991158714294434, 5.256573905944824, Rotation2d.fromDegrees(-120)),
      new Pose2d(13.708047142028809, 5.413857688903809, Rotation2d.fromDegrees(-120)),
    },
    {
      new Pose2d(
          12.448975945229831, 5.391850951541687, Rotation2d.fromDegrees(-57.931551216345206)),
      new Pose2d(12.103763639987946, 5.297896918328212, Rotation2d.fromDegrees(-58.27866028322585)),
    },
    {
      new Pose2d(11.588984260559082, 4.186556816101074, Rotation2d.fromRadians(0)),
      new Pose2d(11.588498611450195, 3.8510172367095947, Rotation2d.fromRadians(0)),
    },
    {
      new Pose2d(12.20763469696045, 2.7914857959747314, Rotation2d.fromDegrees(60)),
      new Pose2d(12.480260620117188, 2.602744827270508, Rotation2d.fromDegrees(60)),
    },
    {
      AllianceFlipUtil.apply(
          new Pose2d(3.8932504449234008789, 5.3975392832508087158, Rotation2d.fromDegrees(-60))),
      AllianceFlipUtil.apply(
          new Pose2d(3.602333632564544678, 5.231181619834899902, Rotation2d.fromDegrees(-60))),
    },
  };

  public double getDistanceToNearestReef(Pose2d pose) {

    if (Constants.AlignTuningMode && tuningTempPose != null) {
      return pose.getTranslation().getDistance(tuningTempPose.getTranslation());
    }

    double mindistance = Double.POSITIVE_INFINITY;
    int index = -1;
    for (int i = 0;
        i < (Constants.CompField ? reefscoringPositionsComp : reefscoringPositionsPractice).length;
        i++) {
      for (int j = 0;
          j
              < (Constants.CompField ? reefscoringPositionsComp : reefscoringPositionsPractice)
                  [i].length;
          j++) {
        if (pose.getTranslation()
                .getDistance(
                    (Constants.CompField ? reefscoringPositionsComp : reefscoringPositionsPractice)
                        [i][j].getTranslation())
            < mindistance) {
          index = i;
          mindistance =
              pose.getTranslation()
                  .getDistance(
                      (Constants.CompField
                              ? reefscoringPositionsComp
                              : reefscoringPositionsPractice)
                          [i][j].getTranslation());
        }
      }
    }
    return mindistance;
  }

  public double getDistanceToNearestCoralStation(Pose2d pose) {
    double mindistance = Double.POSITIVE_INFINITY;
    int index = -1;
    for (int i = 0; i < coralStationPositions.length; i++) {
      if (pose.getTranslation().getDistance(coralStationPositions[i].getTranslation())
          < mindistance) {
        index = i;
        mindistance = pose.getTranslation().getDistance(coralStationPositions[i].getTranslation());
      }
    }

    return mindistance;
  }

  @AutoLogOutput(key = "NearestReefPose")
  public Pose2d getNearestReefPose(Pose2d pose) {

    if (Constants.AlignTuningMode && tuningTempPose != null) {
      return tuningTempPose;
    }

    double mindistance = Double.POSITIVE_INFINITY;
    int index1 = -1;
    int index2 = -1;

    for (int i = 0;
        i < (Constants.CompField ? reefscoringPositionsComp : reefscoringPositionsPractice).length;
        i++) {
      for (int j = 0;
          j
              < (Constants.CompField ? reefscoringPositionsComp : reefscoringPositionsPractice)
                  [i].length;
          j++) {
        if (pose.getTranslation()
                .getDistance(
                    ((Constants.CompField ? reefscoringPositionsComp : reefscoringPositionsPractice)
                        [i][j].getTranslation()))
            < mindistance) {
          index1 = i;
          index2 = j;
          mindistance =
              pose.getTranslation()
                  .getDistance(
                      ((Constants.CompField
                              ? reefscoringPositionsComp
                              : reefscoringPositionsPractice)
                          [i][j].getTranslation()));
        }
      }
    }

    return ((Constants.CompField ? reefscoringPositionsComp : reefscoringPositionsPractice)
        [index1][index2]);
  }

  @AutoLogOutput(key = "NearestReefPose")
  public Pose2d getNearestReefPose(Pose2d pose, BooleanSupplier toggle) {

    if (Constants.AlignTuningMode && tuningTempPose != null) {
      return tuningTempPose;
    }

    double mindistance = Double.POSITIVE_INFINITY;
    int index1 = -1;
    int index2 = -1;

    for (int i = 0;
        i < (Constants.CompField ? reefscoringPositionsComp : reefscoringPositionsPractice).length;
        i++) {
      for (int j = 0;
          j
              < (Constants.CompField ? reefscoringPositionsComp : reefscoringPositionsPractice)
                  [i].length;
          j++) {
        if (pose.getTranslation()
                .getDistance(
                    ((Constants.CompField ? reefscoringPositionsComp : reefscoringPositionsPractice)
                        [i][j].getTranslation()))
            < mindistance) {
          index1 = i;
          index2 = j;
          mindistance =
              pose.getTranslation()
                  .getDistance(
                      ((Constants.CompField
                              ? reefscoringPositionsComp
                              : reefscoringPositionsPractice)
                          [i][j].getTranslation()));
        }
      }
    }

    if (toggle.getAsBoolean()) {
      index2 =
          (Constants.CompField ? reefscoringPositionsComp : reefscoringPositionsPractice)
                  [index1].length
              - 1
              - index2;
    }

    return ((Constants.CompField ? reefscoringPositionsComp : reefscoringPositionsPractice)
        [index1][index2]);
  }

  @AutoLogOutput(key = "NearestReefTagID")
  public int getNearestReefTagID(Pose2d pose) {

    double mindistance = Double.POSITIVE_INFINITY;
    int index1 = -1;
    int index2 = -1;

    for (int i = 0;
        i < (Constants.CompField ? reefscoringPositionsComp : reefscoringPositionsPractice).length;
        i++) {
      for (int j = 0;
          j
              < (Constants.CompField ? reefscoringPositionsComp : reefscoringPositionsPractice)
                  [i].length;
          j++) {
        if (pose.getTranslation()
                .getDistance(
                    ((Constants.CompField ? reefscoringPositionsComp : reefscoringPositionsPractice)
                        [i][j].getTranslation()))
            < mindistance) {
          index1 = i;
          index2 = j;
          mindistance =
              pose.getTranslation()
                  .getDistance(
                      ((Constants.CompField
                              ? reefscoringPositionsComp
                              : reefscoringPositionsPractice)
                          [i][j].getTranslation()));
        }
      }
    }
    return tagList[index1];
  }

  // used for auto-aim towards coral station for aiming
  @AutoLogOutput(key = "NearestCoralStationPose")
  public Pose2d getNearestCoralStationPose(Pose2d pose) {
    double mindistance = Double.POSITIVE_INFINITY;
    int index = -1;

    for (int i = 0; i < coralStationPositions.length; i++) {
      if (pose.getTranslation().getDistance((coralStationPositions[i].getTranslation()))
          < mindistance) {
        index = i;
        mindistance =
            pose.getTranslation().getDistance((coralStationPositions[i]).getTranslation());
      }
    }

    return (coralStationPositions[index]);
  }

  @AutoLogOutput(key = "Swerve/ModuleLimits")
  public ModuleLimits getModuleLimits() {
    // if (DriverStation.isTeleop()) {
    return switch (elevatorPosition) {
      case 0 -> TunerConstants.moduleLimitsFree;

      case 1 -> TunerConstants.moduleLimitsL1Elevator;

      case 2 -> TunerConstants.moduleLimitsL2Elevator;

      case 3 -> TunerConstants.moduleLimitsL3Elevator;

      case 4 -> TunerConstants.moduleLimitsL4Elevator;

      default -> TunerConstants.moduleLimitsL3Elevator;
    };
    // }

  }
}
