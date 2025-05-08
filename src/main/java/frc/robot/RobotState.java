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

  @AutoLogOutput(key = "RobotState/limitSwitching")
  @Getter
  @Setter
  private volatile boolean limitSwitching = true;

  @AutoLogOutput(key = "RobotState/reefAutoAiming")
  @Getter
  @Setter
  private volatile boolean reefAutoAiming = false;

  @AutoLogOutput(key = "RobotState/intakeAutoAiming")
  @Getter
  @Setter
  private volatile boolean intakeAutoAiming = false;

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
  private Pose2d[][] reefscoringPositionsPractice = {
    {
      new Pose2d(
          3.165418872833252 - Units.inchesToMeters(1.75),
          4.173290481567383,
          Rotation2d.fromDegrees(0)),
      new Pose2d(
          3.165418872833252 - Units.inchesToMeters(1.75),
          3.778736753463745,
          Rotation2d.fromDegrees(0))
    },
    {
      new Pose2d(
          3.7052595386505127 - Units.inchesToMeters(1.75) / 2.0,
          2.9254032344818115 - Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
          Rotation2d.fromDegrees(60)),
      new Pose2d(
          3.9753434658050537 - Units.inchesToMeters(4.5) / 2.0,
          2.793973445892334 - Units.inchesToMeters(4.5) * Math.sin(Math.PI / 3),
          Rotation2d.fromDegrees(60)),
    },
    {
      new Pose2d(
          5.0181637210845947 + Units.inchesToMeters(2.25) / 2.0,
          2.803973445892334 - Units.inchesToMeters(2.25) * Math.sin(Math.PI / 3),
          Rotation2d.fromDegrees(120)),
      new Pose2d(
          5.296333312988281 + Units.inchesToMeters(1.75) / 2.0,
          2.947709321975708 - Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
          Rotation2d.fromDegrees(120)),
    },
    {
      new Pose2d(
          5.825868129730225 + Units.inchesToMeters(1.75),
          3.8744309043884277,
          Rotation2d.fromDegrees(180)),
      new Pose2d(
          5.825868129730225 + Units.inchesToMeters(1.75),
          4.183290481567383,
          Rotation2d.fromDegrees(180)),
    },
    {
      new Pose2d(
          5.2896333312988281 + Units.inchesToMeters(1.75) / 2.0,
          5.0910011825561523 + Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
          Rotation2d.fromDegrees(-120)),
      new Pose2d(
          4.9981637210845947 + Units.inchesToMeters(2.3) / 2.0,
          5.247050011825561525 + Units.inchesToMeters(2.3) * Math.sin(Math.PI / 3),
          Rotation2d.fromDegrees(-120)),
    },
    {
      new Pose2d(
          3.9653434658050537 - Units.inchesToMeters(1.75) / 2.0,
          5.255135288238525 + Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
          Rotation2d.fromDegrees(-60)),
      new Pose2d(
          3.6792595386505127 - Units.inchesToMeters(1.75) / 2.0,
          5.095135288238525 + Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
          Rotation2d.fromDegrees(-60)),
    },

    // red alliance, also in order of A, B, C, etc
    {
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              3.165418872833252 - Units.inchesToMeters(1.75),
              4.173290481567383,
              Rotation2d.fromDegrees(0))),
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              3.165418872833252 - Units.inchesToMeters(1.75),
              3.858736753463745,
              Rotation2d.fromDegrees(0)))
    },
    {
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              3.6952595386505127 - Units.inchesToMeters(1.75) / 2.0,
              2.9454032344818115 - Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
              Rotation2d.fromDegrees(60))),
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              3.92053434658050537 - Units.inchesToMeters(1.75) / 2.0,
              2.793973445892334 - Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
              Rotation2d.fromDegrees(60))),
    },
    {
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              5.0181637210845947 + Units.inchesToMeters(2.25) / 2.0,
              2.803973445892334 - Units.inchesToMeters(2.25) * Math.sin(Math.PI / 3),
              Rotation2d.fromDegrees(120))),
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              5.296333312988281 + Units.inchesToMeters(1.75) / 2.0,
              2.947709321975708 - Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
              Rotation2d.fromDegrees(120))),
    },
    {
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              5.825868129730225 + Units.inchesToMeters(1.75),
              3.8744309043884277,
              Rotation2d.fromDegrees(180))),
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              5.825868129730225 + Units.inchesToMeters(1.75),
              4.183290481567383,
              Rotation2d.fromDegrees(180))),
    },
    {
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              5.2896333312988281 + Units.inchesToMeters(1.75) / 2.0,
              5.0910011825561523 + Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
              Rotation2d.fromDegrees(-120))),
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              5.011637210845947 + Units.inchesToMeters(1.85) / 2.0,
              5.2550011825561525 + Units.inchesToMeters(1.85) * Math.sin(Math.PI / 3),
              Rotation2d.fromDegrees(-120))),
    },
    {
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              3.9653434658050537 - Units.inchesToMeters(1.75) / 2.0,
              5.255135288238525 + Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
              Rotation2d.fromDegrees(-60))),
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              3.6792595386505127 - Units.inchesToMeters(1.75) / 2.0,
              5.095135288238525 + Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
              Rotation2d.fromDegrees(-60))),
    },
  };

  // for official fields // not finalized or tuned.
  // DON'T use alliancefliputil to flip to get corresponding red scoring pose2ds THEY ARE ALL HERE
  // AND WILL BE TUNED FOR EACH FIELD
  // blue positions, in the order of A, B, C, etc. Using tuningTempPose to tune on practice day
  private Pose2d[][] reefscoringPositionsComp = {
    {
      new Pose2d(
          3.165418872833252 - Units.inchesToMeters(1.75),
          4.173290481567383,
          Rotation2d.fromDegrees(0)),
      new Pose2d(
          3.165418872833252 - Units.inchesToMeters(1.75),
          3.858736753463745,
          Rotation2d.fromDegrees(0))
    },
    {
      new Pose2d(
          3.6852595386505127 - Units.inchesToMeters(1.75) / 2.0,
          2.9454032344818115 - Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
          Rotation2d.fromDegrees(60)),
      new Pose2d(
          3.9753434658050537 - Units.inchesToMeters(1.75) / 2.0,
          2.793973445892334 - Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
          Rotation2d.fromDegrees(60)),
    },
    {
      new Pose2d(
          5.0181637210845947 + Units.inchesToMeters(2.25) / 2.0,
          2.803973445892334 - Units.inchesToMeters(2.25) * Math.sin(Math.PI / 3),
          Rotation2d.fromDegrees(120)),
      new Pose2d(
          5.296333312988281 + Units.inchesToMeters(1.75) / 2.0,
          2.947709321975708 - Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
          Rotation2d.fromDegrees(120)),
    },
    {
      new Pose2d(
          5.825868129730225 + Units.inchesToMeters(1.75),
          3.8744309043884277,
          Rotation2d.fromDegrees(180)),
      new Pose2d(
          5.825868129730225 + Units.inchesToMeters(1.75),
          4.183290481567383,
          Rotation2d.fromDegrees(180)),
    },
    {
      new Pose2d(
          5.2896333312988281 + Units.inchesToMeters(1.75) / 2.0,
          5.0910011825561523 + Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
          Rotation2d.fromDegrees(-120)),
      new Pose2d(
          5.0381637210845947 + Units.inchesToMeters(1.75) / 2.0,
          5.257050011825561525 + Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
          Rotation2d.fromDegrees(-120)),
    },
    {
      new Pose2d(
          3.9653434658050537 - Units.inchesToMeters(1.75) / 2.0,
          5.255135288238525 + Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
          Rotation2d.fromDegrees(-60)),
      new Pose2d(
          3.6792595386505127 - Units.inchesToMeters(1.75) / 2.0,
          5.095135288238525 + Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
          Rotation2d.fromDegrees(-60)),
    },

    // red alliance, also in order of A, B, C, etc
    {
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              3.165418872833252 - Units.inchesToMeters(1.75),
              4.173290481567383,
              Rotation2d.fromDegrees(0))),
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              3.165418872833252 - Units.inchesToMeters(1.75),
              3.858736753463745,
              Rotation2d.fromDegrees(0)))
    },
    {
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              3.6952595386505127 - Units.inchesToMeters(1.75) / 2.0,
              2.9454032344818115 - Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
              Rotation2d.fromDegrees(60))),
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              3.92053434658050537 - Units.inchesToMeters(1.75) / 2.0,
              2.793973445892334 - Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
              Rotation2d.fromDegrees(60))),
    },
    {
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              5.0181637210845947 + Units.inchesToMeters(2.25) / 2.0,
              2.803973445892334 - Units.inchesToMeters(2.25) * Math.sin(Math.PI / 3),
              Rotation2d.fromDegrees(120))),
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              5.296333312988281 + Units.inchesToMeters(1.75) / 2.0,
              2.947709321975708 - Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
              Rotation2d.fromDegrees(120))),
    },
    {
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              5.825868129730225 + Units.inchesToMeters(1.75),
              3.8744309043884277,
              Rotation2d.fromDegrees(180))),
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              5.825868129730225 + Units.inchesToMeters(1.75),
              4.183290481567383,
              Rotation2d.fromDegrees(180))),
    },
    {
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              5.2896333312988281 + Units.inchesToMeters(1.75) / 2.0,
              5.0910011825561523 + Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
              Rotation2d.fromDegrees(-120))),
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              5.011637210845947 + Units.inchesToMeters(1.85) / 2.0,
              5.2550011825561525 + Units.inchesToMeters(1.85) * Math.sin(Math.PI / 3),
              Rotation2d.fromDegrees(-120))),
    },
    {
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              3.9653434658050537 - Units.inchesToMeters(1.75) / 2.0,
              5.255135288238525 + Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
              Rotation2d.fromDegrees(-60))),
      AllianceFlipUtil.forcedApply(
          new Pose2d(
              3.6792595386505127 - Units.inchesToMeters(1.75) / 2.0,
              5.095135288238525 + Units.inchesToMeters(1.75) * Math.sin(Math.PI / 3),
              Rotation2d.fromDegrees(-60))),
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
