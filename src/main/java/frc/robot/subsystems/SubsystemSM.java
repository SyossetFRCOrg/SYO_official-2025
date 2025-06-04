package frc.robot.subsystems;

public interface SubsystemSM 
{
    // State Machine state enums
    public enum WantedState {};
    public enum SystemState {};

    private void handleStateTransitions() {}
    private void applyCurrentState() {}
}
