package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDs.LEDSection;

import static frc.robot.Constants.LEDs.*;
import static com.techhounds.houndutil.houndlib.leds.LEDPatterns.*;

/**
 * The LED subsystem, which controls the state of the LEDs by superimposing
 * requested LED states and continuously updates the LED's buffer. Other classes
 * can request specific LED states to be active, and they will be applied in
 * priority order.
 * 
 * @author dr
 */
@LoggedObject
public class LEDs extends SubsystemBase {
    /** The LEDs. */
    private AddressableLED leds = new AddressableLED(0);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LENGTH);
    /** Notifier thread for displaying when the robot code is initializing. */
    private final Notifier loadingNotifier;

    private ArrayList<LEDState> currentStates = new ArrayList<LEDState>();

    private static final Color GOLD = new Color("#FF6E05");

    /**
     * An enum of all possible LED states.
     */
    public enum LEDState {
        GOLD_WAVE(
                wave(new Color("#FBBF05"), 25, 10, 50, 150, LEDSection.ALL),
                wave(new Color("#FBBF05"), 25, 10, 50, 150, LEDSection.ALL)),
        DEMO_RED(
                fire2012Palette(0.2, 0.3,
                        List.of(Color.kBlack, Color.kRed),
                        LEDSection.ELEVATOR_LEFT),
                wave(Color.kRed, 25, 15, 0, 255, LEDSection.ELEVATOR_TOP_LEFT),
                wave(Color.kRed, 25, 15, 0, 255, LEDSection.ELEVATOR_TOP_RIGHT),
                fire2012Palette(0.2, 0.3,
                        List.of(Color.kBlack, Color.kRed),
                        LEDSection.ELEVATOR_RIGHT),

                wave(Color.kRed, 25, 15, 0, 255, LEDSection.HOPPER_RIGHT_FULL),
                wave(Color.kRed, 25, 15, 0, 255, LEDSection.HOPPER_LEFT_FULL),

                wave(Color.kRed, 25, 15, 0, 255, LEDSection.HOPPER_ARCH_LEFT),
                wave(Color.kRed, 25, 15, 0, 255, LEDSection.HOPPER_ARCH_RIGHT)),
        DEMO_GOLD(
                fire2012Palette(0.2, 0.3,
                        List.of(Color.kBlack, GOLD),
                        LEDSection.ELEVATOR_LEFT),
                wave(GOLD, 25, 15, 0, 255, LEDSection.ELEVATOR_TOP_LEFT),
                wave(GOLD, 25, 15, 0, 255, LEDSection.ELEVATOR_TOP_RIGHT),
                fire2012Palette(0.2, 0.3,
                        List.of(Color.kBlack, GOLD),
                        LEDSection.ELEVATOR_RIGHT),

                wave(GOLD, 25, 15, 0, 255, LEDSection.HOPPER_RIGHT_FULL),
                wave(GOLD, 25, 15, 0, 255, LEDSection.HOPPER_LEFT_FULL),

                // solid(Color.kGreen, LEDSection.HOPPER_ARCH_LEFT),
                // solid(Color.kYellow, LEDSection.HOPPER_ARCH_RIGHT));
                wave(GOLD, 25, 15, 0, 255, LEDSection.HOPPER_ARCH_LEFT),
                wave(GOLD, 25, 15, 0, 255, LEDSection.HOPPER_ARCH_RIGHT));

        private List<Consumer<AddressableLEDBuffer>> bufferConsumers;

        @SafeVarargs
        private LEDState(Consumer<AddressableLEDBuffer>... bufferConsumer) {
            this.bufferConsumers = Arrays.asList(bufferConsumer);
        }
    }

    /**
     * Initializes the LEDs.
     */
    public LEDs() {
        leds.setLength(LENGTH);
        leds.setData(buffer);
        leds.start();

        loadingNotifier = new Notifier(
                () -> {
                    synchronized (this) {
                        breathe(Color.kWhite, 3, 0, 255, LEDSection.ALL).accept(buffer);
                        leds.setData(buffer);
                    }
                });
        loadingNotifier.startPeriodic(0.02);

        setDefaultCommand(updateBufferCommand());
    }

    /**
     * Creates a command that requests a specific LED state to be active. When
     * command is cancelled, the state will no longer be active.
     * 
     * @param state the state to request
     * @return the command
     */
    public Command requestStateCommand(LEDState state) {
        return Commands.run(() -> currentStates.add(state)).ignoringDisable(true);
    }

    /**
     * Creates a command that updates the LED buffer with the contents of the
     * current LED states.
     * 
     * @return
     */
    public Command updateBufferCommand() {
        return run(() -> {
            loadingNotifier.stop();
            clear();
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

    /** Clears the buffer. */
    public void clear() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kBlack);
        }
    }
}
