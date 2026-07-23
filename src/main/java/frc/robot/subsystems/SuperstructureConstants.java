// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.PhysicalConstants;

public class SuperstructureConstants {    
    public static enum TurretStates {
        /* list of possible operational states the turret can be set to */
        IDLE(0,0,0),
        MANUAL(false, 0),
        AUTOAIM_IDLE(true, 0.3),
        AUTOAIM_SHOOT(true, 1),
        HUB_PREALIGNED_LOCATION(0,0.135,208),
        POINT_DIRECTLY_BACK_FOR_PASSING(0,0.25,250),
        TRENCH_PREALIGNED_LEFT(-1.5, 0.1, 216),
        TRENCH_PREALIGNED_RIGHT(1.5, 0.1, 216),
        REVERSE(0,0, -100);  

        double kTurretAngleRadians;
        double kHoodAngleRadians;
        double kFlywheelRadPerSec;

        boolean kIsAutoAim;
        double kAutoAimFlywheelPercentage;
        
        /** constructor for turret states */
        private TurretStates(
            double turretAngleRadians,
            double hoodAngleRadians,
            double flywheelRadPerSec
        ) {
            this.kIsAutoAim = false;
            this.kTurretAngleRadians = turretAngleRadians;
            this.kHoodAngleRadians = hoodAngleRadians;
            this.kFlywheelRadPerSec = flywheelRadPerSec;

        }
        /**  constructor for autoaim */
        private TurretStates(
            boolean isAutoAim,
            double autoAimFlywheelPercentage
        ) {
            this.kIsAutoAim = isAutoAim;
            this.kAutoAimFlywheelPercentage = autoAimFlywheelPercentage;
            this.kTurretAngleRadians = 0;
            this.kHoodAngleRadians = 0;
            this.kFlywheelRadPerSec = 0;
        }
    }
    /* intitalizes numerical values for the FuelManagmentStates enum */
    public static double kSlapdownUpAngle = 0;
    public static double kSlapdownDownAngle = 1.645;
    public static double kIntakeRollerVelocity = 201;

    public static double kSlapdownShakeUpAngleRad = 1.1;
    public static double kSlapdownShakePeriodSec = 0.8; // time for a full cycle
    
    public static enum IntakeStates {
        /** list of opereational states of the intake and thier inputs */
        RETRACTED(kSlapdownUpAngle,0),
        EXTENDED(kSlapdownDownAngle, 0),
        INTAKING(kSlapdownDownAngle, kIntakeRollerVelocity),
        //INTAKING(true, kSlapdownDownAngle, 1.3, 0.5, kIntakeRollerVoltage),
        SHOOTING(true, kSlapdownDownAngle, kSlapdownShakeUpAngleRad, kSlapdownShakePeriodSec, 200),
        RUN_KICK(true, kSlapdownDownAngle, kSlapdownShakeUpAngleRad, kSlapdownShakePeriodSec, 200),
        RUN_KICK_HIGH(true, kSlapdownDownAngle, 0.5, 1.6, 200),
        REVERSE(kSlapdownDownAngle, -kIntakeRollerVelocity);

        public final double kSlapdownAngle;
        public final boolean kRunSlapdownShake;
        public final double kSlapdownShakeUpAngle;
        public final double kSlapdownShakePeriodSecs;
        public final double kRollerVelocity;

        /** Constructor for intake states */
        private IntakeStates(
            double slapdownAngle,
            double rollerVelocity
        ) {
            this.kSlapdownAngle = slapdownAngle;
            this.kRunSlapdownShake = false;
            this.kSlapdownShakeUpAngle = slapdownAngle;
            this.kSlapdownShakePeriodSecs = Double.MAX_VALUE;
            this.kRollerVelocity = rollerVelocity;
        }

        private IntakeStates(
            boolean runSlapdownShake,
            double shakeDownAngle,
            double slapdownShakeUpAngle,
            double slapdownShakePeriodSecs,
            double rollerVelocity
        ) {
            this.kRunSlapdownShake = runSlapdownShake;
            this.kSlapdownAngle = shakeDownAngle;
            this.kSlapdownShakeUpAngle = slapdownShakeUpAngle;
            this.kSlapdownShakePeriodSecs = slapdownShakePeriodSecs;
            this.kRollerVelocity = rollerVelocity;
        }
    }

    public static double kSpindexerIndexerVelocity = 80;
    public static double kKickerIndexVelocity = 400;
    
    public static enum IndexStates {
        /** list of all possible indexing states and their inputs */
        IDLE(0,0),
        INDEXING(kSpindexerIndexerVelocity, kKickerIndexVelocity),
        REVERSE(-4, -kKickerIndexVelocity);

        public final double kSpindexerVelocity;
        public final double kKickerVelocity;

        /** construcor for indexStates */
        private IndexStates(
            double spindexerVelocity,
            double kickerVelocity
        ) {
            this.kSpindexerVelocity = spindexerVelocity;
            this.kKickerVelocity = kickerVelocity;
        }
    }

    public static final double kClimberDownPosition = 0;
    public static final double kClimberClimbedPosition = 0.1;
    public static final double kClimberUpPosition = 0.4;
    public static final double kHookInPosition = 0;
    public static final double kHookOutPosition = Math.PI;
    public static enum ClimbStates {
        IDLE(kClimberDownPosition, kHookInPosition),
        READY(kClimberUpPosition, kHookInPosition),
        LOCKED(kClimberUpPosition, kHookOutPosition),
        CLIMBED(kClimberClimbedPosition, kHookOutPosition);

        public final double kClimberPosition;
        public final double kHookPosition;
        
        private ClimbStates(double kClimberPosition, double kHookPosition) {
            this.kClimberPosition = kClimberPosition;
            this.kHookPosition = kHookPosition;
        }
    }

    public static final double kTurretMoveSlapdownAngleLimitRad = 1;    
    public static final double kClimbSlapdownMaxAngleRad = 0.5;
    public static final double kMinFlywheelShootingVelocity = 50;
    public static final double kMinSpindexerIndexerVelocity = 65;

    public static final double kLeftPassingTargetYCoordinate = 0.75 * PhysicalConstants.FieldConstants.fieldWidth;
    public static final double kRightPassingTargetYCoordinate = 0.25 * PhysicalConstants.FieldConstants.fieldWidth;

    public record FieldTargets(
        Pose3d kAllianceOrigin,
        Pose3d kHubTarget,
        Pose3d kLeftPassingTarget,
        Pose3d kRightPassingTarget
    ) {

    }

    public static final FieldTargets kBlueAllianceTargets = new FieldTargets(
        new Pose3d(),
        new Pose3d(PhysicalConstants.FieldConstants.Hub.topCenterPoint, Rotation3d.kZero),
        new Pose3d(new Translation3d(1, kLeftPassingTargetYCoordinate, 0), Rotation3d.kZero),
        new Pose3d(new Translation3d(2, kRightPassingTargetYCoordinate, 0), Rotation3d.kZero)
    );

    public static final FieldTargets kRedAllianceTargets = new FieldTargets(
        new Pose3d(
            new Translation3d(PhysicalConstants.FieldConstants.fieldLength, PhysicalConstants.FieldConstants.fieldWidth, 0),
            new Rotation3d(Rotation2d.fromRadians(Math.PI))),
        new Pose3d(PhysicalConstants.FieldConstants.Hub.oppTopCenterPoint, Rotation3d.kZero),
        // Left and right are swapped for red due to rotating 180 degrees
        new Pose3d(new Translation3d(PhysicalConstants.FieldConstants.fieldLength - 2, kRightPassingTargetYCoordinate, 0), Rotation3d.kZero),
        new Pose3d(new Translation3d(PhysicalConstants.FieldConstants.fieldLength - 1, kLeftPassingTargetYCoordinate, 0), Rotation3d.kZero)
    );

    public static final int kAutoAimMaxIterations = 5;
    public static final double kAutoAimTimeToleranceSeconds = 5;

    // Ratios for counting balls
    public static final double kFlywheelVelocityRecoveryRatio = 0.95;
    public static final double kFlywheelVelocityDropRatio = 0.9;
}
