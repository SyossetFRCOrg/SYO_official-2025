package frc.robot;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.*;
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

  // for practice fields like Arumdaun
  private Pose2d[][] reefscoringPositionsPractice = // not finalized or tuned.
      { // DON'T use alliancefliputil to flip to get corresponding red scoring pose2ds THEY ARE ALL
    // HERE AND WILL BE TUNED FOR EACH FIELD

    // blue positions, in the order of A, B, C, etc. Using tuningTempPose to tune on practice day
    // (for practice fields)
    {
      new Pose2d(3.04, 4.24, Rotation2d.fromDegrees(-2.77)),
      new Pose2d(3.085, 3.787, Rotation2d.fromDegrees(0.3))
    },
    {
      new Pose2d(3.662068954711103, 2.7579618237973946, Rotation2d.fromDegrees(55.40424829370364)),
      new Pose2d(
          3.6934151064239247, 3.1333841763974344, Rotation2d.fromDegrees(50.127172524706936)),
    },
    {
      new Pose2d(5.109475612640381, 2.710167169570923, Rotation2d.fromDegrees(120)),
      new Pose2d(5.384280204772949, 2.786501884460449, Rotation2d.fromDegrees(120)),
    },
    {
      new Pose2d(5.91, 3.96, Rotation2d.fromDegrees(-170.46)),
      new Pose2d(5.91, 4.34, Rotation2d.fromDegrees(-168)),
    },
    {
      new Pose2d(5.305, 5.256, Rotation2d.fromDegrees(-120)),
      new Pose2d(4.83, 5.45, Rotation2d.fromDegrees(-105)),
    },
    {
      new Pose2d(3.91, 5.37, Rotation2d.fromDegrees(-60.31)),
      new Pose2d(3.56, 5.22, Rotation2d.fromDegrees(-60)),
    },

    // red alliance, also in order of A, B, C, etc
    {
      new Pose2d(
          14.585301176984437, 3.896938165617065, Rotation2d.fromDegrees(-177.90584757021125)),
      new Pose2d(14.58023245172989, 4.262148436741021, Rotation2d.fromDegrees(-178.18264814361783))
    },
    {
      new Pose2d(
          13.795372583823335, 5.199808161686881, Rotation2d.fromDegrees(-112.19009068279485)),
      new Pose2d(
          13.678263570728163, 5.075483712029739, Rotation2d.fromDegrees(-119.35380053745384)),
    },
    {
      new Pose2d(12.553411573962745, 5.522220199513951, Rotation2d.fromDegrees(-75.56430507216514)),
      new Pose2d(12.220190239970675, 5.313180552341106, Rotation2d.fromDegrees(-66.59963907787123)),
    },
    {
      new Pose2d(11.434800574539622, 4.120487765593584, Rotation2d.fromDegrees(4.258431286310804)),
      new Pose2d(
          11.630359866710407, 3.8171459032191764, Rotation2d.fromDegrees(0.4258799558972908)),
    },
    {
      new Pose2d(12.059619122520424, 2.9011493331040046, Rotation2d.fromDegrees(46.74099652124185)),
      new Pose2d(12.569388978911922, 2.666802085718384, Rotation2d.fromDegrees(59.87328091953917)),
    },
    {
      new Pose2d(13.658877514318094, 2.858955012982499, Rotation2d.fromDegrees(135.41335183195721)),
      new Pose2d(14.14544804013203, 2.993801028186214, Rotation2d.fromDegrees(134.30460726161323)),
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
      new Pose2d(6.0959417724609375, 3.801184997558594, Rotation2d.fromDegrees(180)),
      AllianceFlipUtil.apply(
          new Pose2d(11.588498611450195, 3.8510172367095947, Rotation2d.fromRadians(0)))
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
