package lib.ballistic;

import java.util.function.DoubleSupplier;

/** Class that counts the number of balls that go through a flywheel.
 * 
 * @apiNote Make sure to call update() periodically.
 */
public class BallCounter {
    private int count;

    private boolean enabled;

    private final DoubleSupplier velocitySupplier;
    private final DoubleSupplier targetVelocitySupplier;
    private final double minimumVelocityPercentDrop;

    private double previousVelocity;
    private boolean hasRecovered;

    public BallCounter(DoubleSupplier velocitySupplier, DoubleSupplier targetVelocitySupplier, double minimumVelocityPercentDrop) {
        this.enabled = false;
        this.velocitySupplier = velocitySupplier;
        this.targetVelocitySupplier = targetVelocitySupplier;
        this.minimumVelocityPercentDrop = minimumVelocityPercentDrop;

        this.previousVelocity = velocitySupplier.getAsDouble();
        this.hasRecovered = false;
    }

    public void update() {
        if (enabled) {
            final double currentVelocity = velocitySupplier.getAsDouble();
            final double currentTargetVelocity = targetVelocitySupplier.getAsDouble();

            if (withinTolerance(currentTargetVelocity, currentVelocity, minimumVelocityPercentDrop)) {
                hasRecovered = true;
            } else if (hasRecovered && previousVelocity > currentVelocity) {
                count++;
                hasRecovered = false;
            }

            previousVelocity = currentVelocity;
        }
    }

    public double getCount() {
        return count;
    }

    public void resetCount() {
        count = 0;
    }

    private boolean withinTolerance(double target, double actual, double toleranceAsDecimal) {
        return (target * toleranceAsDecimal) - actual < 0;
    }
}
