package frc.robot.subsystems.kicker;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class KickerConstants {
    // CAN Id
    public static final int kCanId = 44;

    // Voltage compensation and current limit
    public static final double kVoltageCompensation = 10.5;
    public static final int kCurrentLimitAmps = 20;

    public static final boolean kInverted = true;
    public static final IdleMode kIdleMode = IdleMode.kBrake;

    // Velocity conversion factor
    public static final double kGearRatio = 1;
    public static final double kVelocityConversionFactor = 2 * Math.PI / kGearRatio / 60.0;
}
