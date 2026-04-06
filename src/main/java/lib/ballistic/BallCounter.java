package lib.ballistic;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;

/** Class that counts the number of balls that go through a flywheel.
 * 
 * @apiNote Make sure to call update() periodically.
 */
public class BallCounter {
    private int count;

    private boolean enabled;

    private final DoubleSupplier velocitySupplier;
    private final double minimumVelocityPercentDrop;

    //private KalmanFilter<N1, N1, N1> filteredVelocity;

    public BallCounter(DoubleSupplier velocitySupplier, double minimumVelocityPercentDrop) {
        this.enabled = false;
        this.velocitySupplier = velocitySupplier;
        this.minimumVelocityPercentDrop = minimumVelocityPercentDrop;

        /*
        filteredVelocity = new KalmanFilter<N1, N1, N1>(
            Nat.N1(),Nat.N1(), new LinearSystem<>(null, null, null, null),
            VecBuilder.fill(3.0), VecBuilder.fill(0.1), 0.02); */
    }

    public void update() {
        if (enabled) {

        }
    }
}
