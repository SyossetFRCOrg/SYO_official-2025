package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeIntakeConstants;

public class LEDs extends SubsystemBase 
{
    private Spark ledController = new Spark(Constants.LEDsConstants.LED_CONTROLLER_ID);
    
    public void testLED()
    {
        ledController.set(-0.97);
    }
    public void offLED()
    {
        ledController.set(0.99);
    }

    public final Command testLEDCommand = Commands.startEnd( 
            () -> testLED(), 
            () -> ledController.set(0),
            this
    ).withName("struc.testLED");
    
}