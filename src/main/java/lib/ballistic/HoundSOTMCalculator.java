package lib.ballistic;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HoundSOTMCalculator {
    public static CommonSOTMSolution solveShootOnTheFly(
            Pose3d shooterPose,
            Pose3d targetPose,
            ChassisSpeeds fieldRelRobotVelocity,
            // ChassisAccelerations fieldRelRobotAcceleration,
            double targetSpeedRps,
            int maxIterations,
            double timeTolerance) {

        ShotSolution sol = solveBallisticWithSpeed(
                shooterPose,
                targetPose,
                targetSpeedRps);

        double t = sol.flightTimeSeconds();
        Pose3d effectiveTarget = targetPose;

        for (int i = 0; i < maxIterations; i++) {

            double dx = fieldRelRobotVelocity.vxMetersPerSecond * t;
            // + 0.5 * fieldRelRobotAcceleration.axMetersPerSecondSquared * t * t;

            double dy = fieldRelRobotVelocity.vyMetersPerSecond * t;
            // + 0.5 * fieldRelRobotAcceleration.ayMetersPerSecondSquared * t * t;

            effectiveTarget = new Pose3d(
                    targetPose.getX() - dx,
                    targetPose.getY() - dy,
                    targetPose.getZ(),
                    targetPose.getRotation());

            ShotSolution newSol = solveBallisticWithSpeed(
                    shooterPose,
                    effectiveTarget,
                    targetSpeedRps);

            if (Math.abs(newSol.flightTimeSeconds() - t) < timeTolerance) {
                return new CommonSOTMSolution(
                        effectiveTarget,
                        newSol.launchPitchRad(),
                        newSol.launchSpeed(),
                        newSol.flightTimeSeconds(),
                        0);
            }

            sol = newSol;
            t = newSol.flightTimeSeconds();
        }

        return new CommonSOTMSolution(
                effectiveTarget,
                sol.launchPitchRad(),
                sol.launchSpeed(),
                sol.flightTimeSeconds(),
                0);
    }

    public record ShotSolution(
            double launchPitchRad,
            double launchSpeed,
            double flightTimeSeconds) {
    }

    public static ShotSolution solveBallisticWithSpeed(
            Pose3d shooterPose,
            Pose3d targetPose,
            double launchSpeedMPS) {

        Translation3d s = shooterPose.getTranslation();
        Translation3d t = targetPose.getTranslation();

        double dx = t.getX() - s.getX();
        double dy = t.getY() - s.getY();
        double dz = t.getZ() - s.getZ();

        double d = Math.hypot(dx, dy);
        if (d < 1e-9) {
            throw new IllegalArgumentException("Horizontal distance too small");
        }

        double v2 = launchSpeedMPS * launchSpeedMPS;
        double g = 9.81;

        double discriminant = v2 * v2 - g * (g * d * d + 2.0 * dz * v2);
        if (discriminant < 0) {
            return new ShotSolution(0, 0, 0);
        }

        // LOW-ARC solution (use +Math.sqrt(...) for high arc)
        double tanTheta = (v2 + Math.sqrt(discriminant)) / (g * d);

        double launchPitch = Math.atan(tanTheta);

        double vHoriz = launchSpeedMPS * Math.cos(launchPitch);
        double time = d / vHoriz;

        return new ShotSolution(launchPitch, launchSpeedMPS, time);
    }
}
