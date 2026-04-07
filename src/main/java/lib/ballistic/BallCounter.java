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

    private double previousVelocity;
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

        this.previousVelocity = velocitySupplier.getAsDouble();
        this.hasRecovered = false;
    }

    public void update() {
        final double currentVelocity = velocitySupplier.getAsDouble();
        final double currentTargetVelocity = targetVelocitySupplier.getAsDouble();

        if (withinTolerance(currentTargetVelocity, currentVelocity, minimumVelocityPercentDrop)) {
            hasRecovered = true;
        } else if (hasRecovered && previousVelocity > currentVelocity) {
            if (targetIsHubSupplier.getAsBoolean()) {
                hubCount++;
            } else {
                passCount++;
            }
            hasRecovered = false;
        }

        previousVelocity = currentVelocity;
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

    private boolean withinTolerance(double target, double actual, double toleranceAsDecimal) {
        return (target * toleranceAsDecimal) - actual < 0;
    }
}
