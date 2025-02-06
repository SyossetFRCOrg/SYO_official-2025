package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface FeedForward extends Sendable {
    public double calculate(double position, double velocity, double acceleration);

    public default double calculate(double position, double velocity) {
        return calculate(position, velocity, 0.0);
    }

    @Override
    public default void initSendable(SendableBuilder builder) {
        
    }
}
