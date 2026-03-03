// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */


public final class Constants {
    public static class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static class SystemConstants {
        public static final RobotMode kMode = RobotMode.SIM;
        public static final boolean kEnableSignalLogger = false;
        public static final boolean kEnableStatusLogger = false;
        public static final boolean kEnableRTPriority = true;
        public static final boolean kLogOdometry = true;
        public static final double kLoopPeriodMs = 20;
        public static final boolean kEnableSwitchablePDHChannel = true;

        public static enum RobotMode {
            REAL,
            SIM,
            REPLAY;
        }
    }

    /** Contains starting position and team */
    public static class GameConstants {
        public static Alliance teamColor = Alliance.Blue;
        public static AutonSides autonSide = AutonSides.Right;
        public static boolean rearRightCameraEnabledAuton = false; // Only set to true if running algae auton

        // Autonomous command is selected in getAutonomousCommand() in RobotContainer
        
        public enum TeamColors {
            kTeamColorBlue("BLUE"),
            kTeamColorRed("RED");

            public final String color;

            private TeamColors(String color) {
                this.color = color;
            }
        }
        
        // States describing whether the auton is on the left or right side of the alliance
        public enum AutonSides {
            Left(false),
            Right(true);

            public final boolean isRightSide;

            private AutonSides (boolean isRightSide) {
                this.isRightSide = isRightSide;
            }
        }
    }

    public static class CANConstants {
        public static final CANBus kRioBus = new CANBus("rio");
        public static final CANBus kCANivoreBus = new CANBus("Default Name");
    }

    public static class LEDConstants {
        public static final double kHPSignalTime = 3.0;

        public static class LEDDIOConstants {
            public static final int kDIOPort1 = 7;
            public static final int kDIOPort2 = 8;
            public static final int kDIOPort3 = 9; 
        }

        public static class LEDOnRIOConstants {
            public static final int kPWMPort = 0;
            public static final int kLength = 72;

            public static final double kFlashSeconds = 0.1;
            public static final int kEmptyStateBrightness = 100;
            public static final int kFlashingStateBrightness = 100;
        }

        public static enum LEDStates {
            IDLE(false, false, false),
            CORAL_INDEXED(true, false, false),
            HUMAN_PLAYER_SIGNAL(false, true, false),
            ALGAE(true, true, false),
            AUTO_ALIGNED(false, false, true),
            AUTO_ALIGNING(false, false, false),
            OFF(true, false, true),
            ELEVATOR_ZEROED(false, true, true),
            
            RED_HP_SIGNAL(true, true, true),
            ORANGE_HP_SIGNAL(false,false,false),
            YELLOW_HP_SIGNAL(false,false,false),
            GREEN_HP_SIGNAL(false,false,false),
            BLUE_HP_SIGNAL(false,false,false),
            PURPLE_HP_SIGNAL(false,false,false),
            WHITE_HP_SIGNAL(false,false,false);
        }

        public final boolean onesPlace;
        public final boolean twosPlace;
        public final boolean foursPlace;

        private LEDStates(boolean onesPlace, boolean twosPlace, boolean foursPlace) {
            this.onesPlace = onesPlace;
            this.twosPlace = twosPlace;
            this.foursPlace = foursPlace;
        }
    }
}
