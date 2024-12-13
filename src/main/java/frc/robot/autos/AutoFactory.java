package frc.robot.autos;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;

import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.function.BooleanSupplier;

/**
 * A factory for creating autonomous programs for a given {@link Auto}
 */
@SuppressWarnings({"UnusedMethod", "UnusedVariable", "EmptyBlockTag"})
class AutoFactory {
    private static final double AMPBAR_ZERO_DEGREES = 0.0;

    private final DriverStation.Alliance alliance;

    private final RobotContainer robotContainer;
    private final Drive drive;
    private final Superstructure superstructure;
    private boolean trajectoriesLoaded = false;

    /**
     * Create a new <code>AutoFactory</code>.
     *
     * @param robotContainer The {@link RobotContainer}
     */
    AutoFactory(final DriverStation.Alliance alliance, final RobotContainer robotContainer, final Drive drive, final Superstructure superstructure) {
        this.alliance = alliance;
        this.robotContainer = robotContainer;
        this.drive = drive;
        this.superstructure = superstructure;
    }

    /* Autonomous program factories
     *
     * Factory methods should be added here for each autonomous program.
     * The factory methods must:
     *   1. Be package-private (i.e. no access modifier)
     *   2. Accept no parameters
     *   3. Return a link Command
     */

    Command createIdleCommand() {
        return superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.STOPPED);
    }


    
    Command createOneToThreeToC(){
        PathPlannerPath firstSegment = loadSegment(Location.START, Location.ONE);
        // PathPlannerPath pathtry = PathPlannerPath.fromPathFile("New Path");

        preloadTrajectoryClass(firstSegment);
        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(resetPose(firstSegment));
        c.addCommands(subwooferShootInPlace());
        c.addCommands(followWhileIntaking(firstSegment));
        c.addCommands(limelightShootInPlace(2));
        c.addCommands(followWhileIntaking(Location.ONE, Location.TWO));
        c.addCommands(limelightShootInPlace(2));
        c.addCommands(followWhileIntaking(Location.TWO, Location.THREE));
        c.addCommands(limelightShootInPlace(2));
        c.addCommands(followWhileIntaking(Location.THREE, Location.C));
        c.addCommands(followThenShoot(Location.C, Location.SHOOT, 2));
        return c;
    }

    Command createThreeToOneToA(){
        PathPlannerPath firstSegment = loadSegment(Location.START, Location.THREE);

        preloadTrajectoryClass(firstSegment);

        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(resetPose(firstSegment));
        c.addCommands(subwooferShootInPlace());
        c.addCommands(followWhileIntaking(firstSegment));
        c.addCommands(limelightShootInPlace(2));
        c.addCommands(followWhileIntaking(Location.THREE, Location.TWO));
        c.addCommands(limelightShootInPlace(2));
        c.addCommands(followWhileIntaking(Location.TWO, Location.ONE));
        c.addCommands(limelightShootInPlace(2));
        c.addCommands(followWhileIntaking(Location.ONE, Location.A));
        c.addCommands(followThenShoot(Location.A, Location.SHOOT, 2));
        return c;
    }


    Command createMidfieldABC(){
        PathPlannerPath firstSegment = loadSegment(Location.START, Location.A);
        preloadTrajectoryClass(firstSegment);

        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(resetPose(firstSegment));
        c.addCommands(subwooferShootInPlace());
        c.addCommands(followWhileIntaking(firstSegment));
        c.addCommands(followThenShoot(Location.A, Location.SHOOT, 2));        
        c.addCommands(followWhileIntaking(Location.SHOOT, Location.B));
        c.addCommands(followThenShoot(Location.B, Location.SHOOT, 2));
        c.addCommands(followWhileIntaking(Location.SHOOT, Location.C));
        c.addCommands(followThenShoot(Location.C, Location.SHOOT, 2));
       
        return c;
    }

    Command createMidfieldACB(){
        PathPlannerPath firstSegment = loadSegment(Location.START, Location.A);
        preloadTrajectoryClass(firstSegment);

        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(resetPose(firstSegment));
        c.addCommands(subwooferShootInPlace());
        c.addCommands(followWhileIntaking(firstSegment));
        c.addCommands(followThenShoot(Location.A, Location.SHOOT, 2));
        c.addCommands(followWhileIntaking(Location.SHOOT, Location.C));
        c.addCommands(followThenShoot(Location.C, Location.SHOOT, 2));
        c.addCommands(followWhileIntaking(Location.SHOOT, Location.B));
        c.addCommands(followThenShoot(Location.B, Location.SHOOT, 2));
       
        return c;
    }

    Command createMidfieldBAC(){
        PathPlannerPath firstSegment = loadSegment(Location.START, Location.B);

        preloadTrajectoryClass(firstSegment);
        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(resetPose(firstSegment));
        c.addCommands(subwooferShootInPlace());
        c.addCommands(followWhileIntaking(firstSegment));
        c.addCommands(followThenShoot(Location.B, Location.SHOOT, 2));
        c.addCommands(followWhileIntaking(Location.SHOOT, Location.A));
        c.addCommands(followThenShoot(Location.A, Location.SHOOT, 2));
        c.addCommands(followWhileIntaking(Location.SHOOT, Location.C));
        c.addCommands(followThenShoot(Location.C, Location.SHOOT, 2));
       
        return c;
    }


    Command createMidfieldBCA(){
        PathPlannerPath firstSegment = loadSegment(Location.START, Location.B);

        preloadTrajectoryClass(firstSegment);
        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(resetPose(firstSegment));
        c.addCommands(subwooferShootInPlace());
        c.addCommands(followWhileIntaking(firstSegment));
        c.addCommands(followThenShoot(Location.B, Location.SHOOT, 2));        
        c.addCommands(followWhileIntaking(Location.SHOOT, Location.C));
        c.addCommands(followThenShoot(Location.C, Location.SHOOT, 2));
        c.addCommands(followWhileIntaking(Location.SHOOT, Location.A));
        c.addCommands(followThenShoot(Location.A, Location.SHOOT, 2));
       
        return c;
    }


    Command createMidfieldED(){
        PathPlannerPath firstSegment = loadSegment(Location.START, Location.E);

        preloadTrajectoryClass(firstSegment);
        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(resetPose(firstSegment));
        c.addCommands(subwooferShootInPlace());
        c.addCommands(followWhileIntaking(firstSegment));
        c.addCommands(followThenShoot(Location.E, Location.SHOOT, 2));        
        c.addCommands(followWhileIntaking(Location.SHOOT, Location.D));
        c.addCommands(followThenShoot(Location.D, Location.SHOOT, 2));
               
        return c;
    }

    Command createMidfieldDE(){
        PathPlannerPath firstSegment = loadSegment(Location.START, Location.D);

        preloadTrajectoryClass(firstSegment);
        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(resetPose(firstSegment));
        c.addCommands(followWhileIntaking(firstSegment));
        c.addCommands(followThenShoot(Location.D, Location.SHOOT, 2));

        c.addCommands(followWhileIntaking(Location.SHOOT, Location.E));
        c.addCommands(followThenShoot(Location.E, Location.SHOOT, 2));
               
        return c;
    }




    private Command limelightShootInPlace(double sec){
        return deadline(new WaitCommand(sec), superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.LIMELIGHT_SHOT).alongWith(drive.setRotationLock()))
        
                .andThen(drive.disableRotationLock()
                .andThen(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE)));
    }

    private Command limelightShootInPlace(){
        return superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.LIMELIGHT_SHOT).alongWith(drive.setRotationLock())
                .andThen(new WaitUntilCommand((() -> superstructure.flywheel.hasNoteShot()))
                .andThen(drive.disableRotationLock())
                .andThen(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE)));
    }


    private Command subwooferShootInPlace(){
            return superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.SUBWOOFER_SHOT)
                .andThen(new WaitCommand(1.5))
                .andThen(Commands.deadline(waitSeconds(.15), superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE).repeatedly())
                .andThen(drive.disableRotationLock()));
                
    }

    private Command followThenShoot(PathPlannerPath path) {
        return follow(path)
                .alongWith(
                        superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.LIMELIGHT_SHOT).alongWith(drive.setRotationLock()))
                .andThen(new WaitUntilCommand((() -> superstructure.flywheel.hasNoteShot()))
                .andThen(drive.disableRotationLock())
                .andThen(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE)));
    }

    private Command followThenShoot(Location start, Location end) {
        return follow(start, end)
                .alongWith(
                        superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.LIMELIGHT_SHOT).alongWith(drive.setRotationLock()))
                .andThen(new WaitUntilCommand((() -> superstructure.flywheel.hasNoteShot()))
                .andThen(drive.disableRotationLock())
                .andThen(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE)));
    }

    private Command followThenShoot(Location start, Location end, double sec) {
        return follow(start, end)
                .andThen(
                        superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.LIMELIGHT_SHOT).alongWith(drive.setRotationLock()))
                .andThen(new WaitCommand(sec))
                .andThen(drive.disableRotationLock())
                .andThen(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE));
    }

    // Intake piece
    private Command followWhileIntaking(Location start, Location end) {
        return deadline(follow(start, end), superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.INTAKE_DOWN).repeatedly())
        .andThen(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE));
    }
    // Intake piece
    private Command followWhileIntaking(PathPlannerPath path) {
        return deadline(follow(path), superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.INTAKE_DOWN).repeatedly())
        .andThen(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE));
    }

    // Auto init helpers
    private Command resetPose(final PathPlannerPath segment) {
        return runOnce(() -> {
            var correctedTraj = segment.getTrajectory(new ChassisSpeeds(), new Rotation2d());
			Pose2d pose = correctedTraj.getInitialTargetHolonomicPose();
            // Pose2d pose = segment.getPreviewStartingHolonomicPose(); //getpreviewstartingholonomicpose didn't work
            //getStartingDifferentialPose worked!!!
            
            drive.setPose(pose);
        });
        
    }
    // Auto init helpers
    private Command resetPose(final Pose2d pose) {
        return runOnce(() -> {
            // Pose2d pose = segment.getPreviewStartingHolonomicPose(); //getpreviewstartingholonomicpose didn't work
            //getStartingDifferentialPose worked!!!
            
            drive.setPose(pose);
        });
        
    }

    // Path following
    private Command follow(final Location start, final Location end) {
        return follow(loadSegment(start, end));
    }
    // Path following
    private Command follow(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    private void preloadTrajectoryClass(PathPlannerPath firstSegment) {
        // This is done because Java loads classes lazily. Calling this here loads the trajectory class which
        // is used to follow paths and saves user code ms loop time at the start of auto.
        if (!trajectoriesLoaded) {
            trajectoriesLoaded = true;
            var trajectory = new PathPlannerTrajectory(
                    firstSegment,
                    drive.getChassisSpeeds(),
                    drive.getPose().getRotation());
        }
    }

    // Load paths
    private PathPlannerPath loadSegment(final Location start, final Location end) {
        var name = "%STO%S".formatted(start, end);
        var path = PathPlannerPath.fromChoreoTrajectory("%S%S".formatted(alliance, name));
        path.preventFlipping = true;

        // return new AutoSegment(start, end, name, path);
        return path;
    }
}
