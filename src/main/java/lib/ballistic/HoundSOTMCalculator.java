// Credit to TechHOUNDS. This is a modified version of their shoot
// on the fly calulator, which can be found here:
// https://github.com/frc868/houndutil/blob/main/src/main/java/com/techhounds/houndutil/houndlib/ShootOnTheFlyCalculator.java

package lib.ballistic;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

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
                final double launchYawRad = effectiveTarget.relativeTo(shooterPose).getRotation().getZ();

                Logger.recordOutput("Effective Auto-Aim Target", effectiveTarget);
                return new CommonSOTMSolution(
                        sol.launchPitchRad(),
                        launchYawRad,
                        sol.flightTimeSeconds()
                    );
            }

            sol = newSol;
            t = newSol.flightTimeSeconds();
        }

        final double launchYawRad = effectiveTarget.relativeTo(shooterPose).getRotation().getZ();

        Logger.recordOutput("Effective Auto-Aim Target", effectiveTarget);
        return new CommonSOTMSolution(
                sol.launchPitchRad(),
                launchYawRad,
                sol.flightTimeSeconds()
            );
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
            DriverStation.reportError("Horizontal distance for shot calulator is too small.", true);
            return new ShotSolution(0, 0, Double.MAX_VALUE);
        }

        double v2 = launchSpeedMPS * launchSpeedMPS;
        double g = 9.81;

        double discriminant = v2 * v2 - g * (g * d * d + 2.0 * dz * v2);
        if (discriminant < 0) {
            return new ShotSolution(0, 0, Double.MAX_VALUE);
        }

        // HIGH-ARC solution (use -Math.sqrt(...) for low arc)
        double tanTheta = (v2 + Math.sqrt(discriminant)) / (g * d);

        double launchPitch = Math.atan(tanTheta);

        double vHoriz = launchSpeedMPS * Math.cos(launchPitch);
        double time = d / vHoriz;

        return new ShotSolution(launchPitch, launchSpeedMPS, time);
    }
}
