package lib.ballistic;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class BasicLaunchCalculator {
    /** Meters away from hub to hood angle in radians */
    private static final InterpolatingDoubleTreeMap distanceToHoodAngleMap = CommonLookupTable.hubTable.hoodAngle();
    /** Meters away from hub to flywheel speed in radians per second */
    private static final InterpolatingDoubleTreeMap distanceToFlywheelSpeedMap = CommonLookupTable.hubTable.flywheelSpeed();

    /** Calculate what parameters to send to the Superstructure
     * 
     * @param turretPose The pose of the turret
     * @param targetPose The pose of the target (hub)
     */
    public static CommonShotSolution calculate(
        Pose2d turretPose,
        Pose2d targetPose
    ) {
        Translation2d turretToTarget = targetPose.relativeTo(turretPose).getTranslation();

        double turretToTargetDistanceMeters = turretToTarget.getNorm();

        double turretAngleRadians = turretToTarget.getAngle().getRadians();
        double hoodAngleRadians = distanceToHoodAngleMap.get(turretToTargetDistanceMeters);
        double flywheelSpeedRadPerSec = distanceToFlywheelSpeedMap.get(turretToTargetDistanceMeters);

        Logger.recordOutput("ShotCalculator/TurretPose", turretPose);

        Logger.recordOutput("ShotCalculator/TurretToTargetDistance", turretToTargetDistanceMeters);

        Logger.recordOutput("ShotCalculator/Target", targetPose);

        Logger.recordOutput("ShotCalculator/HoodAngle", hoodAngleRadians);
        Logger.recordOutput("ShotCalculator/TurretAngle", turretAngleRadians);
        Logger.recordOutput("ShotCalculator/FlywheelSpeed", flywheelSpeedRadPerSec);

        return new CommonShotSolution(hoodAngleRadians, turretAngleRadians, flywheelSpeedRadPerSec);
    }
}