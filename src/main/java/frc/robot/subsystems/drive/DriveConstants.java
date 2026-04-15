// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final int kOdometryUpdateFrequency = 250;

    public static class DriverControlConstants {
        public static final double singleClutchTranslationFactor = 0.5;
        public static final double singleClutchRotationFactor = 0.5;
        public static final double doubleClutchTranslationFactor = 0.35;
        public static final double doubleClutchRotationFactor = 0.4;
        public static final double FPVClutchTranslationFactor = 0.1;
        public static final double FPVClutchRotationFactor = 0.1;
        public static final double maxTranslationSpeedMPS = 6.0; // 5.0 is default
        public static final double maxRotationSpeedRadPerSec = 8.0; // 5.0 is default
    }

    public static class PathPlannerConstants {
        public static final PathConstraints pathConstraints = new PathConstraints(3, 3, Units.degreesToRadians(360), Units.degreesToRadians(360));
        public static final PathConstraints netConstraints = new PathConstraints(3, 4, Units.degreesToRadians(360), Units.degreesToRadians(360));
        public static final double pathGenerationToleranceMeters = 0.011; // Technically it's anything larger than 0.01, but I'm adding .001 just to be safe
        public static final double LEDpathToleranceMeters = 0.03;

        public static class Control {
            public static final PIDConstants transPID = new PIDConstants(5, 0, 0);
            public static final PIDConstants rotPID = new PIDConstants(5, 0, 0);
        }
    }

    public static class DirectDriveConstants {
        public static final Constraints translationConstraints = new Constraints(2, 2);
        public static final Constraints headingConstraints = new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(360));
    }

    public static class AntiDefenseConstants {
        /** When large imapcts are felt, this is the standard deviations to give to the previous pose */
        public static final Matrix<N3,N1> impactPreviousStateStdDev = VecBuilder.fill(0.3, 0.3, 0.5);
        /** Minimum jerk for an impact to be considered large in m/s^3 */
        public static final double minimumJerk = 1000;

        /** Taps to use for vision only pose linear filter */
        public static final int visionOnlyPoseTaps = 5;
        /** Minimum difference in linear velocity in m/s between between vision and odom to be considered slipping in any direction */
        public static final double minDiffInLinearVel = 3;
        /** Minimum difference in amgular velocity in rad/s between between vision and odom to be considered slipping in any direction */
        public static final double minDiffInAngularVel = 3;
        /** Maximum vision latency to still use vision-only pose in seconds */
        public static final double maxVisionLatency = 0.05;
    }
}
