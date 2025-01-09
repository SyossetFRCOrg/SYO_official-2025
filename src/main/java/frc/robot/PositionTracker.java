package frc.robot;

import java.util.function.Supplier;

public class PositionTracker {
    private Supplier<Double> elevatorPositionSupplier;
    private Supplier<Double> armAngleSupplier;

    public void setElevatorPositionSupplier(Supplier<Double> elevatorPositionSupplier) {
        this.elevatorPositionSupplier = elevatorPositionSupplier;
    }

    public void setArmAngleSupplier(Supplier<Double> armAngleSupplier) {
        this.armAngleSupplier = armAngleSupplier;
    }

    public double getElevatorPosition() {
        return elevatorPositionSupplier.get();
    }

    public double getArmAngle() {
        return armAngleSupplier.get();
    }
}