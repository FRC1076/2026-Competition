package lib.hardware;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;

/** Takes in two encoder readings (one from a primary encoder and one from a secondary encoder),
 *  and updates the primary encoder from the secondary encoder when a fault occurs.
 */
public class ResettingDualEncoder {
    private int currArrLoc;

    private DoubleSupplier primaryEncoder;
    private DoubleSupplier secondaryEncoder;

    private BooleanSupplier shouldResetPrimary;
    private Consumer<Double> primaryEncoderReset;

    private final double[] timestamps;
    private final double[] primaryEncoderReadings;
    private final double[] secondaryEncoderReadings;

    public ResettingDualEncoder(
        int loopCount,
        DoubleSupplier primaryEncoder,
        DoubleSupplier secondaryEncoder,
        BooleanSupplier shouldResetPrimary,
        Consumer<Double> primaryEncoderReset
    ) {
        this.currArrLoc = 0;
        this.primaryEncoder = primaryEncoder;
        this.secondaryEncoder = secondaryEncoder;
        this.shouldResetPrimary = shouldResetPrimary;
        this.primaryEncoderReset = primaryEncoderReset;

        this.timestamps = new double[loopCount];
        this.primaryEncoderReadings = new double[loopCount];
        this.secondaryEncoderReadings = new double[loopCount];
    }

    public void update() {
        if (shouldResetPrimary.getAsBoolean()) {
            // Go to the oldest piece of data saved
            final int oldestArrLoc = (currArrLoc + 1) % timestamps.length;

            final double deltaSecondary = secondaryEncoder.getAsDouble() - secondaryEncoderReadings[oldestArrLoc];
            final double newPrimary = primaryEncoderReadings[oldestArrLoc] + deltaSecondary;
            primaryEncoderReset.accept(newPrimary);
        }

        currArrLoc++;
        if (currArrLoc == timestamps.length) {
            currArrLoc = 0;
        }

        timestamps[currArrLoc] = Timer.getFPGATimestamp();
        primaryEncoderReadings[currArrLoc] = primaryEncoder.getAsDouble();
        secondaryEncoderReadings[currArrLoc] = secondaryEncoder.getAsDouble();
    }
}
