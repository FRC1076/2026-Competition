package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.PhysicalConstants;

public class SuperstructureConstants {

    
    public static enum TurretStates {
        /* list of possible operational states the turret can be set to */
        IDLE(0,0,0),
        MANUAL(false, 0),
        AUTOAIM_IDLE(true, 0.75),
        AUTOAIM_SHOOT(true, 1),
        HUB_PREALIGNED_LOCATION(0,0,0),
        POINT_DIRECTLY_BACK_FOR_PASSING(0,0,0),
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
    /** intitalizes numerical values for the FuelManagmentStates enum */
    public static double kSlapdownUpAngle = Math.PI/2;
    public static double kSlapdownDownAngle = 0;
    public static double kIntakeRollerVoltage = 6;
    
    public static enum IntakeStates {
        /** list of opereational states of the intake and thier inputs */
        RETRACTED(kSlapdownUpAngle,0),
        EXTENDED(kSlapdownDownAngle, 0),
        INTAKING(kSlapdownDownAngle, kIntakeRollerVoltage),
        REVERSE(kSlapdownDownAngle, -kIntakeRollerVoltage);

        public final double kSlapdownAngle;
        public final double kRollerVoltage;

        /** Constructor for intake states */
        private IntakeStates(
            double slapdownAngle,
            double rollerVoltage
        ) {
            this.kSlapdownAngle = slapdownAngle;
            this.kRollerVoltage = rollerVoltage;
        }
    }

    public static double kSpindexerIndexerVoltage = 12;
    public static double kKickerIndexVoltage = 6;
    
    public static enum IndexStates {
        /** list of all possible indexing states and their inputs */
        IDLE(0,0),
        INDEXING(kSpindexerIndexerVoltage, kKickerIndexVoltage),
        REVERSE(-kSpindexerIndexerVoltage, -kKickerIndexVoltage);

        public final double kSpindexerVoltage;
        public final double kKickerVoltage;

        /** construcor for indexStates */
        private IndexStates(
            double spindexerVoltage,
            double kickerVoltage
        ) {
            this.kSpindexerVoltage = spindexerVoltage;
            this.kKickerVoltage = kickerVoltage;
        }
    }
    
    

    public static final double kLeftPassingTargetYCoordinate = 0.75 * PhysicalConstants.FieldConstants.fieldWidth;
    public static final double kRightPassingTargetYCoordinate = 0.25 * PhysicalConstants.FieldConstants.fieldWidth;

    public static final Pose3d kAllianceOrigin = 
        Constants.GameConstants.teamColor == Alliance.Blue
            ? new Pose3d()
            : new Pose3d(
                new Translation3d(PhysicalConstants.FieldConstants.fieldLength, PhysicalConstants.FieldConstants.fieldWidth, 0),
                new Rotation3d(Rotation2d.fromRadians(Math.PI)));
    public static final Pose3d kHubTarget = 
        Constants.GameConstants.teamColor == Alliance.Blue
            ? new Pose3d(PhysicalConstants.FieldConstants.Hub.topCenterPoint, Rotation3d.kZero)
            : new Pose3d(PhysicalConstants.FieldConstants.Hub.oppTopCenterPoint, Rotation3d.kZero);
    public static final Pose3d kLeftPassingTarget =
        Constants.GameConstants.teamColor == Alliance.Blue
            ? new Pose3d(new Translation3d(0, kLeftPassingTargetYCoordinate, 0), Rotation3d.kZero)
            : new Pose3d(new Translation3d(PhysicalConstants.FieldConstants.fieldLength, kLeftPassingTargetYCoordinate, 0), Rotation3d.kZero);
    public static final Pose3d kRightPassingTarget =
        Constants.GameConstants.teamColor == Alliance.Blue
            ? new Pose3d(new Translation3d(0, kRightPassingTargetYCoordinate, 0), Rotation3d.kZero)
            : new Pose3d(new Translation3d(PhysicalConstants.FieldConstants.fieldLength, kRightPassingTargetYCoordinate, 0), Rotation3d.kZero);
         

    public static final int kAutoAimMaxIterations = 5;
    public static final double kAutoAimTimeToleranceSeconds = 5;

    public static final InterpolatingDoubleTreeMap kDistanceToFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    static {
        /**  Distance in meters, speed in rad/sec */
        kDistanceToFlywheelSpeedMap.put(1.0, 300.0);
        kDistanceToFlywheelSpeedMap.put(4.0, 500.0);
    }
}
