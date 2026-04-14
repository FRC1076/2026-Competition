package lib.ballistic;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lib.ballistic.CommonLookupTable.ShotTable;

public class SOTMLaunchCalculator {
    /** Latency for loop time */
    private static final double latencySecs = 0.02;
    /** Number of iterations recurse through the time of flight calculation */
    private static final int iterations = 5;

    private static final LinearFilter distanceFilter = LinearFilter.movingAverage(5);

    public static CommonShotSolution calculate(
        Pose2d turretPose,
        Pose2d targetPose,
        ChassisSpeeds turretVelocity,
        ShotTable shotTable
    ) {
        Pose2d virtualTarget = targetPose;
        Translation2d turretToTarget = new Translation2d();

        double turretToTargetDistanceMeters = 0;
        double timeOfFlightSecs = 0;

        for (int i = 0; i < iterations; i++) {
            turretToTarget = virtualTarget.relativeTo(turretPose).getTranslation();
            turretToTargetDistanceMeters = turretToTarget.getNorm();
            timeOfFlightSecs = shotTable.timeOfFlight().get(turretToTargetDistanceMeters);

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

        if (turretVelocity.vyMetersPerSecond + turretVelocity.vyMetersPerSecond + (turretVelocity.omegaRadiansPerSecond / 2) > 0.5) {
            distanceFilter.calculate(turretToTargetDistanceMeters);
        } else {
            turretToTargetDistanceMeters = distanceFilter.calculate(turretToTargetDistanceMeters);
        }

        double turretAngleRadians = turretToTarget.getAngle().getRadians();
        double hoodAngleRadians = shotTable.hoodAngle().get(turretToTargetDistanceMeters);
        double flywheelSpeedRadPerSec = shotTable.flywheelSpeed().get(turretToTargetDistanceMeters);

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

    public static CommonShotSolution calculateHub(
        Pose2d turretPose,
        Pose2d targetPose,
        ChassisSpeeds turretVelocity
    ) {
        return calculate(turretPose, targetPose, turretVelocity, CommonLookupTable.hubTable);
    }

    public static CommonShotSolution calculatePass(
        Pose2d turretPose,
        Pose2d targetPose,
        ChassisSpeeds turretVelocity
    ) {
        return calculate(turretPose, targetPose, turretVelocity, CommonLookupTable.passingTable);
    }
}
