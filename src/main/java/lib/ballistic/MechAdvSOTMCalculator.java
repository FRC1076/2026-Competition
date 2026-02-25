// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package lib.ballistic;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.SystemConstants;

import org.littletonrobotics.junction.Logger;

    public class MechAdvSOTMCalculator {
    private static final LinearFilter turretAngleFilter =
        LinearFilter.movingAverage((int) (0.1 / (SystemConstants.kLoopPeriodMs / 1000)));
    private static final LinearFilter hoodAngleFilter =
        LinearFilter.movingAverage((int) (0.1 / (SystemConstants.kLoopPeriodMs / 1000)));

    private static Rotation2d lastTurretAngle;
    private static double lastHoodAngle;
    private static Rotation2d turretAngle;
    private static double hoodAngle = Double.NaN;
    private static double turretVelocity;
    private static double hoodVelocity;

    public record ShootingParameters(
        boolean isValid,
        Rotation2d turretAngle,
        double turretVelocity,
        double hoodAngle,
        double hoodVelocity,
        double flywheelSpeed) {}

    
    private static double minDistance;
    private static double maxDistance;
    private static double phaseDelay;
    private static final InterpolatingTreeMap<Double, Double> shotHoodAngleMap = CommonLookupTable.distanceToHoodAngleMap;
    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap = CommonLookupTable.distanceToFlywheelSpeedMap;
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = CommonLookupTable.distanceToTimeOfFlightMap;

    public static CommonShotSolution calculate(
        Pose2d turretPose,
        Pose2d targetPose,
        ChassisSpeeds robotRelativeTurretVelocity,
        Rotation2d robotHeading
    ) {
        // Calculate estimated pose while accounting for phase delay
        turretPose =
            turretPose.exp(
                new Twist2d(
                    robotRelativeTurretVelocity.vxMetersPerSecond * phaseDelay,
                    robotRelativeTurretVelocity.vyMetersPerSecond * phaseDelay,
                    robotRelativeTurretVelocity.omegaRadiansPerSecond * phaseDelay));

        // Calculate field-relative robot velocity
        ChassisSpeeds fieldRelativeTurretVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeTurretVelocity, robotHeading);

        // Calculate field relative turret velocity
        double turretVelocityX = fieldRelativeTurretVelocity.vxMetersPerSecond;
        double turretVelocityY = fieldRelativeTurretVelocity.vyMetersPerSecond;

        // Account for imparted velocity by robot (turret) to offset
        double timeOfFlight;
        Pose2d lookaheadPose = turretPose;
        double lookaheadTurretToTargetDistance = targetPose.getTranslation().getDistance(lookaheadPose.getTranslation());
        for (int i = 0; i < 20; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            lookaheadPose =
                new Pose2d(
                    turretPose.getTranslation().minus(new Translation2d(offsetX, offsetY)),
                    turretPose.getRotation());
            lookaheadTurretToTargetDistance = targetPose.getTranslation().getDistance(lookaheadPose.getTranslation());
        }

        // Calculate parameters accounted for imparted velocity
        // turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
        turretAngle = targetPose.minus(lookaheadPose).getTranslation().getAngle();
        hoodAngle = shotHoodAngleMap.get(lookaheadTurretToTargetDistance);
        if (lastTurretAngle == null) lastTurretAngle = turretAngle;
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
        turretVelocity =
            turretAngleFilter.calculate(
                turretAngle.plus(lastTurretAngle).getRadians() / (SystemConstants.kLoopPeriodMs / 1000));
        hoodVelocity =
            hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / (SystemConstants.kLoopPeriodMs / 1000));
        lastTurretAngle = turretAngle;
        lastHoodAngle = hoodAngle;
        ShootingParameters latestParameters =
            new ShootingParameters(
                lookaheadTurretToTargetDistance >= minDistance
                    && lookaheadTurretToTargetDistance <= maxDistance,
                turretAngle,
                turretVelocity,
                hoodAngle,
                hoodVelocity,
                shotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

        // Log calculated values
        Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

        Logger.recordOutput("ShotCalculator/HoodAngle", latestParameters.hoodAngle);
        Logger.recordOutput("ShotCalculator/TurretAngle", latestParameters.turretAngle.getRadians());
        Logger.recordOutput("ShotCalculator/FlywheelSpeed", latestParameters.flywheelSpeed);

        return new CommonShotSolution(latestParameters.hoodAngle, latestParameters.turretAngle.getRadians(), latestParameters.flywheelSpeed);
    }
}