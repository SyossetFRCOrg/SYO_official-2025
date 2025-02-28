package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public interface State<E> {
    public Command runEvent(E event);
}
