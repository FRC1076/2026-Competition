// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class Constants {
    public static class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static class SystemConstants {
        public static final RobotMode kMode = RobotMode.REAL;
        public static final boolean kEnableSignalLogger = false;
        public static final boolean kEnableRTPriority = true;

        public static enum RobotMode {
            REAL,
            SIM,
            REPLAY;
        }
    }

    public static class FlywheelConstants {
        public static final int kMotorPort = 43;
        public static final int kServoPort = 8;

        public static final int kCurrentLimit = 40;

        public static final double kManualFlywheelVolts = 12;
        public static final double kManualReverseVolts = 4;

        public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

        public static final double kServoAngleUpRad = ((4*Math.PI / 3) + (Math.PI/36));
        public static final double kServoAngleDownRad = ((2*Math.PI / 3) + (Math.PI/36));

        public static class Control {
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;

            public static final double kS = 0;
            public static final double kV = 0.025;
            public static final double kA = 0;

            public static final double kMaxAcceleration = 2000;
            public static final double kMaxJerk = 160000;
        }

        public static class ControlSim {
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;

            public static final double kS = 0;
            public static final double kV = 0.02;
            public static final double kA = 0;
        }
    }
}
