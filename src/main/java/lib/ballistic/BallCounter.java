package lib.ballistic;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Class that counts the number of balls that go through a flywheel.
 * 
 * @apiNote Make sure to call update() periodically.
 */
public class BallCounter {
    private int hubCount;
    private int passCount;

    private final DoubleSupplier velocitySupplier;
    private final DoubleSupplier targetVelocitySupplier;
    private final double minimumVelocityPercentDrop;

    private final BooleanSupplier targetIsHubSupplier;

    private boolean hasRecovered;

    public BallCounter(
        DoubleSupplier velocitySupplier,
        DoubleSupplier targetVelocitySupplier,
        double minimumVelocityPercentDrop,
        BooleanSupplier targetIsHubSupplier
    ) {
        this.velocitySupplier = velocitySupplier;
        this.targetVelocitySupplier = targetVelocitySupplier;
        this.minimumVelocityPercentDrop = minimumVelocityPercentDrop;
        this.targetIsHubSupplier = targetIsHubSupplier;

        this.hasRecovered = false;
    }

    public void update() {
        final double currentVelocity = velocitySupplier.getAsDouble();
        final double currentTargetVelocity = targetVelocitySupplier.getAsDouble();

        final double velocityRatio = currentVelocity / currentTargetVelocity;

        if (velocityRatio > (1-minimumVelocityPercentDrop)) {
            hasRecovered = true;
        }

        if (hasRecovered && velocityRatio < (1-minimumVelocityPercentDrop)) {
            if (targetIsHubSupplier.getAsBoolean()) {
                hubCount++;
            } else {
                passCount++;
            }
            hasRecovered = false;
        }
    }

    public int getHubCount() {
        return hubCount;
    }

    public int getPassCount() {
        return passCount;
    }

    public void resetCount() {
        hubCount = 0;
        passCount = 0;
    }
}
