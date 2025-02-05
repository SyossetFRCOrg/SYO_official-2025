package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Consumer;

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

    private ArrayList<LEDState> currentStates = new ArrayList<LEDState>();
    private static final ArrayList<LEDState> DEFAULT_STATES = new ArrayList<LEDState>();

    //
    public enum LEDState 
    {
        DEMO_RED(

        ),
        DEMO_BLACK(

        ),
        DEMO_RAINBOW(

        );

        
        private ArrayList<Consumer<AddressableLEDBuffer>> bufferConsumers = new ArrayList<>();
        private LEDState(Consumer<AddressableLEDBuffer>... consumers) {
            for (Consumer<AddressableLEDBuffer> consumer : consumers) {
                bufferConsumers.add(consumer);
            }
        }
    }

    public Command updateBufferCommand() {
        return run(() -> {
            this.clearLEDs();
            // default LED states
            currentStates.addAll(DEFAULT_STATES);
            currentStates.sort((s1, s2) -> s2.ordinal() - s1.ordinal());
            currentStates.forEach(s -> s.bufferConsumers.forEach(c -> c.accept(buffer)));
            leds.setData(buffer);
            currentStates.clear();
        })
                .ignoringDisable(true)
                .withName("leds.updateBuffer");
    }

    public void clearLEDs()
    {
        for (int i=0; i<buffer.getLength(); i++)
        {
            buffer.setLED(i, Color.kBlack);
        }

    }

}