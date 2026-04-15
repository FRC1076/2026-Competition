package lib.ballistic;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;

/** Class that counts the number of balls that go through a flywheel.
 * 
 * @apiNote Make sure to call update() periodically.
 */
public class BallCounter {
    private int hubCount;
    private int passCount;

    private final DoubleSupplier velocitySupplier;
    private final DoubleSupplier targetVelocitySupplier;
    private final double velocityDropThresholdRatio;
    private final double recoveryThresholdRatio;

    private final BooleanSupplier targetIsHubSupplier;

    private boolean hasRecovered;

    public BallCounter(
        DoubleSupplier velocitySupplier,
        DoubleSupplier targetVelocitySupplier,
        double velocityDropThresholdRatio,
        double recoveryThresholdRatio,
        BooleanSupplier targetIsHubSupplier
    ) {
        this.velocitySupplier = velocitySupplier;
        this.targetVelocitySupplier = targetVelocitySupplier;
        this.velocityDropThresholdRatio = velocityDropThresholdRatio;
        this.recoveryThresholdRatio = recoveryThresholdRatio;
        this.targetIsHubSupplier = targetIsHubSupplier;

        this.hasRecovered = false;
    }

    public void update() {
        final double currentVelocity = velocitySupplier.getAsDouble();
        final double currentTargetVelocity = targetVelocitySupplier.getAsDouble();

        if (currentTargetVelocity <= 1e-6) {
            hasRecovered = false;
            return;
        }


        final double velocityRatio = MathUtil.clamp(currentVelocity / currentTargetVelocity, 0, 1.5);

        if (velocityRatio > recoveryThresholdRatio) {
            hasRecovered = true;
        }

        if (hasRecovered && velocityRatio < velocityDropThresholdRatio) {
            if (targetIsHubSupplier.getAsBoolean()) {
                hubCount++;
            } else {
                passCount++;
            }
            hasRecovered = false;
        }

        Logger.recordOutput("FuelCount/VelocityRatio", velocityRatio);
        Logger.recordOutput("FuelCount/FlywheelHasRecovered", hasRecovered);
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
