package frc.robot.subsystems;

import java.util.stream.Stream;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class StructureCommand extends Command {
    public static class Entry {
        public final Subsystem subsystem;
        public final Command command;

        public Entry(Subsystem subsystem, Command command) {
            this.subsystem = subsystem;
            this.command = command;
        }
    }

    public static Entry command(Subsystem subsystem, Command command) {
        return new Entry(subsystem, command);
    }

    private final Entry[] entries;

    public StructureCommand(Entry... entries) {
        this.entries = entries;
        addRequirements(Stream.of(entries).map(entry -> entry.subsystem).toList());
    }

    @Override
    public void initialize() {
        Stream.of(entries).forEach(entry -> CommandScheduler.getInstance().schedule(entry.command));
    }

    @Override
    public boolean isFinished() {
        return Stream.of(entries).allMatch(entry -> entry.command.isFinished());
    }
    
    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
