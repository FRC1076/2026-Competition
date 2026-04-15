package lib.data;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import lib.utils.GeometryUtils;

public class Pose2dFilter {
    private final LinearFilter xFilter;
    private final LinearFilter yFilter;

    private final LinearFilter sinFilter;
    private final LinearFilter cosFilter;

    private Pose2d currentPose;
    private Pose2d previousPose;

    private double currentTimestamp = 0;
    private double previousTimestamp = 0;

    public Pose2dFilter(int taps) {
        xFilter = LinearFilter.movingAverage(taps);
        yFilter = LinearFilter.movingAverage(taps);
        
        sinFilter = LinearFilter.movingAverage(taps);
        cosFilter = LinearFilter.movingAverage(taps);

        currentPose = new Pose2d();
        previousPose = new Pose2d();
    }

    public void update(Pose2d rawPose, double timestamp) {
        if (timestamp < currentTimestamp) {
            return; // Reject old data
        }

        double x = xFilter.calculate(rawPose.getX());
        double y = yFilter.calculate(rawPose.getY());

        double sin = sinFilter.calculate(rawPose.getRotation().getSin());
        double cos = cosFilter.calculate(rawPose.getRotation().getCos());

        if (timestamp - previousTimestamp >= 0.02) {
            // Only update the previous pose if at least one loop has passed
            // TODO: is the time delta too small
            previousPose = currentPose;
            previousTimestamp = timestamp;
        }
        currentPose = new Pose2d(x, y, new Rotation2d(Math.atan2(sin, cos)));
        currentTimestamp = timestamp;
    }

    public Pose2d getFilteredPose() {
        return currentPose;
    }

    public Pose2d getFilteredVelocityAdjustedPose() {
        double timeDelta = currentTimestamp - previousTimestamp;
        if (timeDelta > 0.1) {
            return currentPose; // Use the current pose if there was a large gap in vision poses
        }

        Twist2d velocity = GeometryUtils.getVelocityBetweenPoses(currentPose, previousPose, timeDelta);
        double latency = getLatency();

        return currentPose.exp(
            new Twist2d(
                velocity.dx * latency,
                velocity.dy * latency,
                velocity.dtheta * latency
            )
        );
    }

    public ChassisSpeeds getSpeeds() {
        return GeometryUtils.twoPosesToChassisSpeeds(currentPose, previousPose, currentTimestamp - previousTimestamp);
    }

    public double getLatency() {
        return Timer.getFPGATimestamp() - currentTimestamp;
    }
}
