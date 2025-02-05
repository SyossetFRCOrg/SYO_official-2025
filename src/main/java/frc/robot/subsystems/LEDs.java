package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* Relevant Documentation for LED Customization
 * https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/AddressableLED.html#%3Cinit%3E(int)
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html#instantiating-the-addressableled-object
 */

public class LEDs extends SubsystemBase 
{
    // Creating the LEDs and buffer objects
    private AddressableLED leds;
    private AddressableLEDBuffer buffer;

    public LEDs() 
    {
        // initializing the LEDs and buffer objects
        leds = new AddressableLED(0); // TODO: Change the port number to the correct one
        buffer = new AddressableLEDBuffer(60); // TODO: Change the number of LEDs to the correct one
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
    }


    public void clearLEDs()
    {
        for (int i=0; i<buffer.getLength(); i++)
        {
            buffer.setLED(i, Color.kBlack);
        }

    }

    public void periodic() 
    {
        leds.setData(buffer);
    }

}