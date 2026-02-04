// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
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
        public static final RobotMode kMode = RobotMode.REAL;
        public static final boolean kEnableSignalLogger = false;
        public static final boolean kEnableRTPriority = true;
        public static final boolean kLogOdometry = true;
        public static final double kLoopPeriodMs = 20;

        public static enum RobotMode {
            REAL,
            SIM,
            REPLAY;
        }
    }

    /** Contains starting position and team */
    public static class GameConstants {
        public static Alliance teamColor = Alliance.Blue;
        public static AutonSides autonSide = AutonSides.Left;
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

    public static class VisionConstants {
        public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        public static class PhotonVision {
            public static final Vector<N3> kDefaultSingleTagStdDevs = VecBuilder.fill(1, 1, 2);
            public static final Vector<N3> kDefaultMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);

            public static enum PhotonConfig {
                // TEST_CAMERA_ONE(
                //     "TEST_CAMERA_1",
                //     kDefaultMultiTagStdDevs,
                //     kDefaultMultiTagStdDevs,
                //     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                //     PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                //     8.5, 10.75, 3,
                //     0, 0, 45
                // );
                ; // Single semicolon to allow for constructor below,
                // empty for now as there are no camera

                public final String name;
                public final Transform3d offset;
                public final Matrix<N3, N1> defaultSingleTagStdDevs;
                public final Matrix<N3, N1> defaultMultiTagStdDevs;
                public final PoseStrategy multiTagPoseStrategy;
                public final PoseStrategy singleTagPoseStrategy;
                private PhotonConfig(
                    String name, 
                    Matrix<N3, N1> defaultSingleTagStdDevs,
                    Matrix<N3, N1> defaultMultiTagStdDevs,
                    PoseStrategy multiTagPoseStrategy,
                    PoseStrategy singleTagPoseStrategy,
                    double xInch, double yInch, double zInch, 
                    double rollDeg, double pitchDeg, double yawDeg
                ) {
                    this.name = name;
                    this.offset = new Transform3d(
                        Units.inchesToMeters(xInch),
                        Units.inchesToMeters(yInch),
                        Units.inchesToMeters(zInch),
                        new Rotation3d(
                            Units.degreesToRadians(rollDeg),
                            Units.degreesToRadians(pitchDeg),
                            Units.degreesToRadians(yawDeg)
                        )
                    );
                    this.multiTagPoseStrategy = multiTagPoseStrategy;
                    this.singleTagPoseStrategy = singleTagPoseStrategy;
                    this.defaultMultiTagStdDevs = defaultMultiTagStdDevs;
                    this.defaultSingleTagStdDevs = defaultSingleTagStdDevs;
                }
            }
        }
    }

    public static class FieldConstants {
        public static final double fieldWidthMeters = Units.inchesToMeters(317.7); // Distance from one edge of the field to the other
        public static final double fieldLengthMeters = Units.inchesToMeters(651.2); // Distance in meters from one drive station to the other side
    }
}
