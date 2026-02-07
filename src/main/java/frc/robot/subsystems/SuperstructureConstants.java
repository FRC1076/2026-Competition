package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.PhysicalConstants;

public class SuperstructureConstants {

    
    public static enum TurretStates {
        /** list of possible operational states the turret can be set to */
        IDLE(0,0,0),
        MANUAL(false),
        AUTOAIM(true),
        HUB_PREALIGNED_LOCATION(0,0,0),
        POINT_DIRECTLY_BACK_FOR_PASSING(0,0,0);  

        /** numeric states of turret (initializes values)*/
        double kTurretAngleRadians;
        double kHoodAngleRadians;
        double kFlywheelRadPerSec;
        boolean kIsAutoAim;
        
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
            boolean isAutoAim
        ) {
            this.kIsAutoAim = isAutoAim;
            this.kTurretAngleRadians = 0;
            this.kHoodAngleRadians = 0;
            this.kFlywheelRadPerSec = 0;
        }
    }
    /** intitalizes numerical values for the FuelManagmentStates enum */
    public static double kSlapdownUpAngle = Math.PI/2;
    public static double kSlapdownDownAngle = 0;
    public static double kIntakeRollerVoltage = 6;
    public static double kSpindexerIndexerVoltage = 12;
    public static double kKickerIndexVoltage = 6;
    /** sets operational states for the FuelManagmentStates */
    public static enum FuelManagementStates {
        IDLE_RETRACTED(kSlapdownUpAngle, 0, 0, 0),
        IDLE_EXTENDED(kSlapdownDownAngle, 0, 0, 0),
        INTAKE_FUEL(kSlapdownDownAngle, kIntakeRollerVoltage, 0, 0),
        INTAKE_INDEX_FUEL(kSlapdownDownAngle, kIntakeRollerVoltage, kSpindexerIndexerVoltage, kKickerIndexVoltage),
        INDEX_FUEL_RETRACTED(kSlapdownUpAngle, 0, kSpindexerIndexerVoltage, kKickerIndexVoltage),
        INDEX_FUEL_EXTENDED(kSlapdownDownAngle, 0, kSpindexerIndexerVoltage, kKickerIndexVoltage);

        /**  initializes variables for fuelmanagement constructor below */
        public final double slapdownRadians;
        public final double rollerVoltage;
        public final double spindexerVoltage;
        public final double kickerVoltage;

        /** constructor for FuelManagmentStates */
        private FuelManagementStates(
            double slapdownRadians,
            double rollerVoltage,
            double spindexerVoltage,
            double kickerVoltage
        ) {
            this.slapdownRadians = slapdownRadians;
            this.rollerVoltage = rollerVoltage;
            this.spindexerVoltage = spindexerVoltage;
            this.kickerVoltage = kickerVoltage;

        }
    }
    public static final double kLeftPassingTargetYCoordinate = 0.75 * PhysicalConstants.FieldConstants.fieldWidth;
    public static final double kRightPassingTargetYCoordinate = 0.25 * PhysicalConstants.FieldConstants.fieldWidth;

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
