// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.drive;

import frc.robot.PhysicalConstants.FieldConstants;
import frc.robot.commands.drive.PPDriveToPose;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.Elastic;
import frc.robot.subsystems.drive.DriveConstants.AntiDefenseConstants;
import frc.robot.subsystems.drive.DriveConstants.PathPlannerConstants;
import lib.data.Pose2dFilter;
import lib.vision.VisionLocalizationSystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

public class DriveSubsystem extends SubsystemBase {
    private final DriveIO io;
    private final DriveIOInputsAutoLogged driveInputs = new DriveIOInputsAutoLogged();
    //private final ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
    //private final ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
    //private final ModuleIOInputsAutoLogged rearLeftInputs = new ModuleIOInputsAutoLogged();
    //private final ModuleIOInputsAutoLogged rearRightInputs = new ModuleIOInputsAutoLogged();
    // private Boolean hasSetAlliance = false; // Wait until the driverstation had an alliance before setting it
    private boolean isAutoAligned = false;
    public final DriveCommandFactory CommandBuilder;
    private final VisionLocalizationSystem vision;
    private final Elastic elastic;

    /** Record to hold the acceleration of the drivetrain */
    private record LinearAccelerationState(
        double xAccel,
        double yAccel,
        double zAccel
    ) {
        @SuppressWarnings("unused")
        private double getXYAccel() {
            return Math.hypot(xAccel, yAccel);
        }

        @SuppressWarnings("unused")
        private double getAcceleration() {
            return Math.sqrt((xAccel * xAccel) + (yAccel * yAccel) + (zAccel * zAccel));
        }
    }

    private final Pose2dFilter visionOnlyPoseFilter;
    private LinearAccelerationState currentAccel;
    private LinearAccelerationState previousAccel;
    private boolean isSlipping;
    private boolean impactDetected;

    public DriveSubsystem(DriveIO io, VisionLocalizationSystem vision, Elastic elastic) {
        this.io = io;
        this.vision = vision;
        this.elastic = elastic;

        visionOnlyPoseFilter = new Pose2dFilter(AntiDefenseConstants.visionOnlyPoseTaps);
        vision.registerMeasurementConsumer(
            (poseEstimate, timestampSeconds, StdDevs) -> {
                io.addVisionMeasurement(poseEstimate, timestampSeconds, StdDevs);
                visionOnlyPoseFilter.update(poseEstimate, timestampSeconds);
            }
        );

       setAllianceRotation(elastic.getSelectedTeamColor());

        io.resetHeading();
        
        CommandBuilder = new DriveCommandFactory(this);   
        
        currentAccel = new LinearAccelerationState(0, 0, 0);
        previousAccel = new LinearAccelerationState(0, 0, 0);
    }

    @Override
    public void periodic(){
        // updateModuleInputs and processInputs are only used for logging
        io.periodic(); //currently just for calling sim
        vision.update(); // TODO: change to updateAll() if desired
        io.updateInputs(driveInputs);
        // io.updateModuleInputs(frontLeftInputs, 0);
        // io.updateModuleInputs(frontRightInputs, 1);
        // io.updateModuleInputs(rearLeftInputs, 2);
        // io.updateModuleInputs(rearRightInputs, 3);
        Logger.processInputs("Drive", driveInputs);
        // Logger.processInputs("Drive/FrontLeft", frontLeftInputs);
        // Logger.processInputs("Drive/FrontRight", frontRightInputs);
        // Logger.processInputs("Drive/RearLeft", rearLeftInputs);
        // Logger.processInputs("Drive/RearRight", rearRightInputs);

         /*
            if(DriverStation.getAlliance().isPresent()){
                hasSetAlliance = true;
                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                    io.setAllianceRotation(Rotation2d.fromDegrees(180));
                } else {
                    io.setAllianceRotation(Rotation2d.fromDegrees(0));
                }
            } 
        }*/
        elastic.updateField(driveInputs.Pose); // Update the robot's pose on Elastic

        // Reduce pose jumping under defense
        currentAccel = new LinearAccelerationState(driveInputs.gyroAccelerationX, driveInputs.gyroAccelerationY, driveInputs.gyroAccelerationZ);
        
        final double currentJerk = Math.abs(currentAccel.getAcceleration() - previousAccel.getAcceleration()); 
        if (currentJerk > AntiDefenseConstants.minimumJerk) {
            // We just had a big impact, so the pose probably didn't move that much as odometry and the cameras think
            io.addVisionMeasurement(getPose(), Timer.getFPGATimestamp(), AntiDefenseConstants.impactPreviousStateStdDev);
            impactDetected = true;
        } else {
            impactDetected = false;
        }

        ChassisSpeeds odometrySpeeds = getChassisSpeeds();
        ChassisSpeeds visionSpeeds = visionOnlyPoseFilter.getSpeeds();
        if (
            visionOnlyPoseFilter.getLatency() <= AntiDefenseConstants.maxVisionLatency && (
                odometrySpeeds.vxMetersPerSecond - visionSpeeds.vxMetersPerSecond > AntiDefenseConstants.minDiffInLinearVel ||
                odometrySpeeds.vyMetersPerSecond - visionSpeeds.vyMetersPerSecond > AntiDefenseConstants.minDiffInLinearVel ||
                odometrySpeeds.omegaRadiansPerSecond - visionSpeeds.omegaRadiansPerSecond > AntiDefenseConstants.minDiffInAngularVel
            )
        ) {
            // Odom is moving much faster than vision
            isSlipping = true;
        } else {
            isSlipping = false;
        }

        Logger.recordOutput("Drive/ImpactDetected", impactDetected);
        Logger.recordOutput("Drive/IsSlipping", isSlipping);
        Logger.recordOutput("Drive/Jerk", currentJerk);
    }

    public void configureAutoBuilder() {
        try {
            AutoBuilder.configure(
                () -> elastic.getPathPlannerMirrored() 
                    ? this.getMirroredPose() 
                    : this.getPose(),
                (pose) -> {
                    if (elastic.getPathPlannerMirrored()) {
                        this.resetPoseMirrored(pose);
                    } else {
                        this.resetPose(pose);
                    }
                },
                () -> driveInputs.Speeds,
                (speeds, feedforwards) -> {
                    if (elastic.getPathPlannerMirrored()) {
                        driveCoMirrored(speeds);
                    } else {
                        driveCO(speeds);
                    }
                },
                new PPHolonomicDriveController(
                    // PID constants for translation
                    PathPlannerConstants.Control.transPID,
                    // PID constants for rotation
                    PathPlannerConstants.Control.rotPID
                ),
                RobotConfig.fromGUISettings(),
                () -> elastic.getSelectedTeamColor() == Alliance.Red,//DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );

            elastic.buildAutoChooser();
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /** This method is not used in any command logic. It is only used for LEDs and Elastic */
    public boolean isAutoAligned() {
        return isAutoAligned;
    }

    public void clearAutoAlignedStatus() {
        isAutoAligned = false;
    }

    /** Swerve drive request with chassis-oriented chassisSpeeds */
    public void driveCO(ChassisSpeeds speeds) {
        io.acceptRequest(new ApplyRobotSpeeds().withSpeeds(speeds));
    }

    /** Swerve drive request with chassis-orriented chassisSpeeds, but mirrors the speeds (used for mirroring paths in auton) */
    public void driveCoMirrored(ChassisSpeeds speeds) {
        speeds.vyMetersPerSecond *= (-1);  
        speeds.omegaRadiansPerSecond *= (-1);
        io.acceptRequest(new ApplyRobotSpeeds().withSpeeds(speeds));
    }

    public double getVelocityMPS() {
        double velMPS = Math.sqrt(Math.pow(driveInputs.Speeds.vxMetersPerSecond,2.0) + Math.pow(driveInputs.Speeds.vyMetersPerSecond,2.0));
        return velMPS;
    }

    public double getAngularVelocityRadPerSec() {
        return driveInputs.Speeds.omegaRadiansPerSecond;
    }

    /** Swerve drive request with field-oriented chassisSpeeds */
    public void driveFO(ChassisSpeeds speeds) {
        io.acceptRequest(new ApplyFieldSpeeds().withSpeeds(speeds).withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));
    }
    
    //TODO: ADD DRIVE METHOD TO DRIVE WITH PATHPLANNER WHEELFORCE FEEDFORWARDS
    
    /** Swerve drive request with heading lock */
    public void driveFOHeadingLocked(double xMetersPerSecond, double yMetersPerSecond, Rotation2d targetDirection) {
        FieldCentricFacingAngle request = new FieldCentricFacingAngle()
        .withVelocityX(xMetersPerSecond)
        .withVelocityY(yMetersPerSecond)
        .withTargetDirection(targetDirection)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
        request.HeadingController.setPID(3.5, 0, 0);
        request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        io.acceptRequest(request);
    }

    /** Set swerveRequest */
    public void acceptRequest(SwerveRequest request) {
        io.acceptRequest(request);
    }

    /** Resets the pose of the robot */
    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }

    /** Resets the pose of the robot, but mirrors the input Pose2d, while remaining on the same side of the field (used for mirroring paths in auton) */
    public void resetPoseMirrored(Pose2d pose) {
        Pose2d newPose = new Pose2d(
            pose.getX(),
            FieldConstants.fieldWidth - pose.getY(),
            pose.getRotation().unaryMinus());
        io.resetPose(newPose);
    }

    /** Makes the current heading of the robot the default zero degree heading
     * (Used if forward is the wrong direction)
     */
    public void resetHeading() {
        io.resetHeading();
    }

    /** Allows you to select which way is forward based on the alliance */
    public void setAllianceRotation(Alliance alliance) {
        if (alliance == Alliance.Red) {
            io.setAllianceRotation(Rotation2d.fromDegrees(180));
        } else {
            io.setAllianceRotation(Rotation2d.fromDegrees(0));
        }
    }

    public Rotation2d getHeading() {
        return io.getPose().getRotation();
    }

    @AutoLogOutput
    public Pose2d getPose() {
        return io.getPose();
    }

    @AutoLogOutput
    public ChassisSpeeds getChassisSpeeds() {
        return io.getChassisSpeeds();
    }

    @AutoLogOutput
    public Pose2d getVisionOnlyPose() {
        return visionOnlyPoseFilter.getFilteredVelocityAdjustedPose();
    }

    @AutoLogOutput
    public ChassisSpeeds getVisionOnlySpeeds() {
        return visionOnlyPoseFilter.getSpeeds();
    }

    @AutoLogOutput
    public Pose2d getSlippingAdjustedPose() {
        /*
        return isSlipping
            ? getVisionOnlyPose()
            : getPose(); */
            return getPose();
    }

    @AutoLogOutput
    public ChassisSpeeds getSlippingAdjustedSpeeds() {
        /*
        return isSlipping
            ? getVisionOnlySpeeds()
            : getChassisSpeeds();*/
        return getChassisSpeeds();
    }

    /** Returns the pose of the robot, but mirrored */
    public Pose2d getMirroredPose() {
        Pose2d pose = new Pose2d(
            io.getPose().getX(),
            FieldConstants.fieldWidth - io.getPose().getY(),
            io.getPose().getRotation().unaryMinus());
        return pose;
    }

    public class DriveCommandFactory {
        private final DriveSubsystem drive;

        private DriveCommandFactory(DriveSubsystem drive) {
            this.drive = drive;
        }

        public Command pathfindToPose(Pose2d targetPose) {
            return AutoBuilder.pathfindToPose(
                targetPose,
                PathPlannerConstants.pathConstraints,
                0.0
            );
        }
    
        public Command followPath(PathPlannerPath path){
            return AutoBuilder.followPath(path);
        }

        public TeleopDriveCommand teleopDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
            return new TeleopDriveCommand(drive, xSupplier, ySupplier, omegaSupplier);
        }

        public Command directDriveToPose(Pose2d targetPose) {
            return Commands.parallel(
                new PPDriveToPose(drive, targetPose),
                Commands.sequence(
                    Commands.runOnce(() -> {isAutoAligned = false;}),
                    Commands.waitUntil(() -> {return targetPose.getTranslation().getDistance(drive.getPose().getTranslation()) < PathPlannerConstants.LEDpathToleranceMeters;}),
                    Commands.runOnce(() -> {isAutoAligned = true;})
                ));
            /*
            DirectDriveToPoseCommand directDriveToPoseCommand = new DirectDriveToPoseCommand(drive, targetPose);

            return Commands.parallel(
                    directDriveToPoseCommand,
                    Commands.sequence(
                        Commands.runOnce(() -> {isAutoAligned = false;}),
                        Commands.waitUntil(() -> {return targetPose.getTranslation().getDistance(drive.getPose().getTranslation()) < PathPlannerConstants.LEDpathToleranceMeters;}),
                        Commands.run(() -> {isAutoAligned = true;})
                    )

            );*/
        }

        public Command directDriveToPose(Pose2d targetPose, PathConstraints constraints) {
            return Commands.parallel(
                new PPDriveToPose(drive, targetPose, constraints, 0),
                Commands.sequence(
                    Commands.runOnce(() -> {isAutoAligned = false;}),
                    Commands.waitUntil(() -> {return targetPose.getTranslation().getDistance(drive.getPose().getTranslation()) < PathPlannerConstants.LEDpathToleranceMeters;}),
                    Commands.runOnce(() -> {isAutoAligned = true;})
                ));
        }
        
        public Command applySwerveRequest(Supplier<SwerveRequest> requestSupplier) {
            return run(() -> acceptRequest(requestSupplier.get()));
        }

    }

}