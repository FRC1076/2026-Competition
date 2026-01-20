package lib.vision;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

/** Class intended to create noise on demand to simulate defense */
public class DefenseNoiseSimulator implements CameraLocalizer {
    // Pose to destroy
    private Supplier<Pose2d> realRobotPoseSupplier;

    // Impact related stuff
    private int remainingNoiseLoops = 0;
    private int totalNoiseLoops = 0;
    private double impactMagnitude = 0.5; // 0-1

    // Standard deviations
    private final Vector<N3> standardDeviations = VecBuilder.fill(0.1,0.1,0.1);

    public DefenseNoiseSimulator(Supplier<Pose2d> realRobotPose) {
        this.realRobotPoseSupplier = realRobotPose;
    }

    @Override
    public String getName() {
        return "Defense Noise Simulator";
    }

    /** Add an impact to the simulator
     * 
     * @param timeInLoops How many robot loops the simulator should last for
     * @param maxMagnitude Maximum total displacement of robot,
     * where 1 caps at one meter of displacement and half a radian of displacement
     */
    public void startImpact(int timeInLoops, double maxMagnitude) {
        remainingNoiseLoops = timeInLoops;
        totalNoiseLoops = timeInLoops;
        impactMagnitude = maxMagnitude;
    }

    @Override
    public Optional<CommonPoseEstimate> getPoseEstimate() {
        if (remainingNoiseLoops <= 0) {
            return Optional.empty();
        } else {
            double xShift = (Math.random() - 0.5) * impactMagnitude * (1.0 / totalNoiseLoops) * 2;
            double yShift = (Math.random() - 0.5) * impactMagnitude * (1.0 / totalNoiseLoops) * 2;
            double thetaShift = (Math.random() - 0.5) * impactMagnitude * (1.0 / totalNoiseLoops);

            Logger.recordOutput("X Shift + Y Shify + Theta Shift", xShift + yShift + thetaShift);

            Pose2d newPose = realRobotPoseSupplier.get()
                    .transformBy(new Transform2d(xShift, yShift, Rotation2d.fromRadians(thetaShift)));

            remainingNoiseLoops--;

            return Optional.of(new CommonPoseEstimate(
                newPose,
                Timer.getFPGATimestamp(),
                standardDeviations));
        }
    }
}
