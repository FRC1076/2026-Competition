package lib.ballistic;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class BasicLaunchCalculator {
    /** Meters away from hub to hood angle in radians */
    private static final InterpolatingDoubleTreeMap distanceToHoodAngleMap = new InterpolatingDoubleTreeMap();
    /** Meters away from hub to flywheel speed in radians per second */
    private static final InterpolatingDoubleTreeMap distanceToFlywheelSpeedMap = new InterpolatingDoubleTreeMap();

    static {
        distanceToHoodAngleMap.put(1.0, 0.0);
        distanceToHoodAngleMap.put(2.0, 0.05);
        distanceToHoodAngleMap.put(3.0, 0.09);
        distanceToHoodAngleMap.put(4.0, 0.12);
        distanceToHoodAngleMap.put(5.0, 0.16);
        distanceToHoodAngleMap.put(6.0, 0.2);

        distanceToFlywheelSpeedMap.put(1.0, 240.0);
        distanceToFlywheelSpeedMap.put(2.0, 250.0);
        distanceToFlywheelSpeedMap.put(3.0, 275.0);
        distanceToFlywheelSpeedMap.put(4.0, 290.0);
        distanceToFlywheelSpeedMap.put(5.0, 305.0);
        distanceToFlywheelSpeedMap.put(6.0, 320.0);
    }

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

        Logger.recordOutput("ShotCalculator/TurretToTargetDistance", turretToTargetDistanceMeters);

        Logger.recordOutput("ShotCalculator/HoodAngle", hoodAngleRadians);
        Logger.recordOutput("ShotCalculator/TurretAngle", turretAngleRadians);
        Logger.recordOutput("ShotCalculator/FlywheelSpeed", flywheelSpeedRadPerSec);

        return new CommonShotSolution(hoodAngleRadians, turretAngleRadians, flywheelSpeedRadPerSec);
    }
}