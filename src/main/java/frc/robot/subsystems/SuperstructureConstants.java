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
        IDLE,
        MANUAL,
        AUTOAIM,
        HUB_PREALIGNED_LOCATION,
        POINT_DIRECTLY_BACK_FOR_PASSING;  
    }

    public static enum FuelManagementStates {
        IDLE_EXTENDED,
        IDLE_RETRACTED,
        INDEX_FUEL_EXTENDED,
        INDEX_FUEL_RETRACTED,
        INTAKE_FUEL,
        INTAKE_INDEX_FUEL;
    }

    public static final double kLeftPassingTargetYCoordinate = 0.75 * PhysicalConstants.FieldConstants.fieldWidth;
    public static final double kRightPassingTargetYCoordinate = 0.25 * PhysicalConstants.FieldConstants.fieldWidth;

    public static final Pose3d kHubTarget = 
        Constants.GameConstants.teamColor == Alliance.Blue
            ? new Pose3d(PhysicalConstants.FieldConstants.Hub.topCenterPoint, Rotation3d.kZero)
            : new Pose3d(PhysicalConstants.FieldConstants.Hub.oppTopCenterPoint, Rotation3d.kZero).relativeTo(null);
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
        // Distance in meters, speed in rad/sec
        kDistanceToFlywheelSpeedMap.put(1.0, 300.0);
        kDistanceToFlywheelSpeedMap.put(4.0, 500.0);
    }
}
