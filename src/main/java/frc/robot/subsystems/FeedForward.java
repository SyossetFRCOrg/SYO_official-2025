package frc.robot.subsystems;

public interface FeedForward {
    public double calculate(double position, double velocity, double acceleration);

    public default double calculate(double position, double velocity) {
        return calculate(position, velocity, 0.0);
    }
}
