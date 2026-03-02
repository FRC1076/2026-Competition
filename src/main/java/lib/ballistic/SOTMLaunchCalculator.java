package lib.ballistic;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SOTMLaunchCalculator {
    /** Meters away from hub to hood angle in radians */
    private static final InterpolatingDoubleTreeMap distanceToHoodAngleMap = CommonLookupTable.distanceToHoodAngleMap;
    /** Meters away from hub to flywheel speed in radians per second */
    private static final InterpolatingDoubleTreeMap distanceToFlywheelSpeedMap = CommonLookupTable.distanceToFlywheelSpeedMap;
    /** Meters away from hub to time of flight in seconds */
    private static final InterpolatingDoubleTreeMap distanceToTimeOfFlightMap = CommonLookupTable.distanceToTimeOfFlightMap;

    /** Latency for loop time */
    private static final double latencySecs = 0.02;
    /** Number of iterations recurse through the time of flight calculation */
    private static final int iterations = 3;

    public static CommonShotSolution calculate(
        Pose2d turretPose,
        Pose2d targetPose,
        ChassisSpeeds turretVelocity
    ) {
        Pose2d virtualTarget = targetPose;
        Translation2d turretToTarget = new Translation2d();

        double turretToTargetDistanceMeters = 0;
        double timeOfFlightSecs = 0;

        for (int i = 0; i < iterations; i++) {
            turretToTarget = virtualTarget.relativeTo(turretPose).getTranslation();
            turretToTargetDistanceMeters = turretToTarget.getNorm();
            timeOfFlightSecs = distanceToTimeOfFlightMap.get(turretToTargetDistanceMeters);

            // Shift the actual target by the velocity times time of flight
            virtualTarget = targetPose.relativeTo(
                new Pose2d(
                    new Translation2d(
                        turretVelocity.vxMetersPerSecond * (timeOfFlightSecs + latencySecs),
                        turretVelocity.vyMetersPerSecond * (timeOfFlightSecs + latencySecs)
                    ),
                    new Rotation2d()
                )
            );
        }

        double turretAngleRadians = turretToTarget.getAngle().getRadians();
        double hoodAngleRadians = distanceToHoodAngleMap.get(turretToTargetDistanceMeters);
        double flywheelSpeedRadPerSec = distanceToFlywheelSpeedMap.get(turretToTargetDistanceMeters);

        Logger.recordOutput("ShotCalculator/TurretPose", turretPose);
        Logger.recordOutput("ShotCalculator/TurretVelocity", turretVelocity);

        Logger.recordOutput("ShotCalculator/TurretToTargetDistance", turretToTargetDistanceMeters);

        Logger.recordOutput("ShotCalculator/Target", virtualTarget);

        Logger.recordOutput("ShotCalculator/TimeOfFlight", timeOfFlightSecs);

        Logger.recordOutput("ShotCalculator/HoodAngle", hoodAngleRadians);
        Logger.recordOutput("ShotCalculator/TurretAngle", turretAngleRadians);
        Logger.recordOutput("ShotCalculator/FlywheelSpeed", flywheelSpeedRadPerSec);

        return new CommonShotSolution(hoodAngleRadians, turretAngleRadians, flywheelSpeedRadPerSec);
    }
}
