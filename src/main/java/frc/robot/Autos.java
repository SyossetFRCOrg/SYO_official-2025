package frc.robot;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.techhounds.houndutil.houndauto.AutoRoutine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CoralSim.CoralSimLocation;
import frc.robot.CoralSim.CoralSimScoreLocation;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class Autos {
    public static AutoRoutine testPath(Drivetrain drivetrain) {
        PathPlannerPath path = PathPlannerPath.fromPathFile("testPath");
        Command command = drivetrain.followPathCommand(path);
        return new AutoRoutine("testPath", command, List.of(path), path.getStartingDifferentialPose());
    }

    public static Command simulateCoral(CoralSimScoreLocation scoreLocation, CoralSim coralSim) {
        return Commands.sequence(
                coralSim.setLocationCommand(CoralSimLocation.HIDDEN),
                coralSim.addScoringLocationCommand(scoreLocation));
    }

    public static AutoRoutine GDC(Drivetrain drivetrain, Elevator elevator, Arm arm, CoralSim coralSim) {
        PathPlannerPath Start_GPath = PathPlannerPath.fromPathFile("Start-G");
        PathPlannerPath G_PickupPath = PathPlannerPath.fromPathFile("G-Pickup");
        PathPlannerPath Pickup_DPath = PathPlannerPath.fromPathFile("Pickup-D");
        PathPlannerPath D_PickupPath = PathPlannerPath.fromPathFile("D-Pickup");
        PathPlannerPath Pickup_CPath = PathPlannerPath.fromPathFile("Pickup-C");

        Command command = Commands.sequence(
                RobotCommands.prepareCoralScoreCommand(ScoreLevel.L4, elevator, arm, coralSim)
                        .alongWith(Commands.waitSeconds(0.5).andThen(drivetrain.followPathCommand(Start_GPath))),
                Commands.parallel(
                        RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, coralSim),
                        Commands.waitSeconds(0.1).andThen(simulateCoral(CoralSimScoreLocation.G_L4,
                                coralSim))),
                drivetrain.followPathCommand(G_PickupPath)
                        .alongWith(RobotCommands.prepareIntakeCoralCommand(elevator, arm, coralSim)),
                coralSim.setLocationCommand(CoralSimLocation.INTAKE),
                Commands.parallel(
                        Commands.waitSeconds(0.1)
                                .andThen(RobotCommands.intakeIntoScoreCommand(ScoreLevel.L4, elevator, arm, coralSim)),
                        drivetrain.followPathCommand(Pickup_DPath)),
                Commands.parallel(
                        RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, coralSim),
                        Commands.waitSeconds(0).andThen(simulateCoral(CoralSimScoreLocation.D_L4,
                                coralSim))),
                drivetrain.followPathCommand(D_PickupPath)
                        .alongWith(RobotCommands.prepareIntakeCoralCommand(elevator, arm, coralSim)),
                coralSim.setLocationCommand(CoralSimLocation.INTAKE),
                Commands.parallel(
                        Commands.waitSeconds(0)
                                .andThen(RobotCommands.intakeIntoScoreCommand(ScoreLevel.L4, elevator, arm, coralSim)),
                        drivetrain.followPathCommand(Pickup_CPath)),
                Commands.parallel(
                        RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, coralSim),
                        Commands.waitSeconds(0.1).andThen(simulateCoral(CoralSimScoreLocation.C_L4,
                                coralSim))));

        return new AutoRoutine("GDC", command,
                List.of(Start_GPath, G_PickupPath, Pickup_DPath, D_PickupPath, Pickup_CPath),
                Start_GPath.getStartingDifferentialPose());
    }
}
