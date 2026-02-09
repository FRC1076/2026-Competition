// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.commands.drive;

import frc.robot.subsystems.drive.DriveSubsystem;

import static frc.robot.subsystems.drive.DriveConstants.DirectDriveConstants.headingConstraints;
import static frc.robot.subsystems.drive.DriveConstants.DirectDriveConstants.translationConstraints;
import static frc.robot.subsystems.drive.DriveConstants.DriverControlConstants.FPVClutchRotationFactor;
import static frc.robot.subsystems.drive.DriveConstants.DriverControlConstants.FPVClutchTranslationFactor;
import static frc.robot.subsystems.drive.DriveConstants.DriverControlConstants.doubleClutchRotationFactor;
import static frc.robot.subsystems.drive.DriveConstants.DriverControlConstants.doubleClutchTranslationFactor;
import static frc.robot.subsystems.drive.DriveConstants.DriverControlConstants.maxRotationSpeedRadPerSec;
import static frc.robot.subsystems.drive.DriveConstants.DriverControlConstants.maxTranslationSpeedMPS;
import static frc.robot.subsystems.drive.DriveConstants.DriverControlConstants.singleClutchRotationFactor;
import static frc.robot.subsystems.drive.DriveConstants.DriverControlConstants.singleClutchTranslationFactor;

import lib.control.ProfiledPIDHolonomicController;
import lib.functional.TriFunction;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

public class TeleopDriveCommand extends Command {
    // The raw speed suppliers, unaffected by the clutches. A reference to these is maintained in order to make applying and unapplying clutches easier
    private final DoubleSupplier rawXSupplier;
    private final DoubleSupplier rawYSupplier;
    private final DoubleSupplier rawOmegaSupplier;

    // The required drive subsystem
    private final DriveSubsystem m_drive;

    // The clutch factors being used
    private double transClutch = 1.0;
    private double rotClutch = 1.0;

    // The actual speed suppliers, these are affected by the clutch factor
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier omegaSupplier;

    private final ProfiledPIDController translationPIDController = new ProfiledPIDController(0.5, 0, 0, translationConstraints); // PathPlanner uses 5, 0, 0
    private final ProfiledPIDController rotationPIDController = new ProfiledPIDController(0.5, 0, 0, headingConstraints); // 5, 0, 0

    private final ProfiledPIDHolonomicController m_DriveController = 
        new ProfiledPIDHolonomicController(
            translationPIDController,
            rotationPIDController
        );

    private SwerveRequest.FieldCentric defaultDriveRequest;
    private SwerveRequest.FieldCentricFacingAngle defaultHeadingLockedDriveRequest;

    // Request Generator declarations

    // The request generator is a function that takes three doubles as parameters and returns a SwerveRequest.
    // During every loop, the requestGenerator is invoked with the values of the three doublesuppliers as parameters,
    // And the resulting SwerveRequest is applied to the Drivetrain
    private TriFunction<Double, Double, Double, SwerveRequest> requestGenerator;

    // A swerve request override. If this optional is not empty, it signals to the TeleopDriveCommand that the request generator has been overridden, and will not
    // generate a default requestGenerator during reloadCommand()
    // The presence of a requestGeneratorOverride supersedes the presence of a headingSupplier
    private Optional<TriFunction<Double, Double, Double, SwerveRequest>> requestGeneratorOverride = Optional.empty();

    // A heading override. If this optional is not empty, it signals to the TeleopDriveCommand that the heading has been locked,
    // And will generate a requestGenerator that returns a FieldCentricFacingAngle request
    private Optional<Supplier<Rotation2d>> headingSupplier = Optional.empty();

    public TeleopDriveCommand(DriveSubsystem drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        m_drive = drive;
        rawXSupplier = () -> xSupplier.getAsDouble() * maxTranslationSpeedMPS;
        rawYSupplier = () -> ySupplier.getAsDouble() * maxTranslationSpeedMPS;
        rawOmegaSupplier = () -> omegaSupplier.getAsDouble() * maxRotationSpeedRadPerSec;
        defaultDriveRequest = new FieldCentric().withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
        defaultHeadingLockedDriveRequest = new FieldCentricFacingAngle()
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withHeadingPID(5.0,0.0,0.0);
        defaultHeadingLockedDriveRequest.HeadingController.enableContinuousInput(-Math.PI,Math.PI);
        addRequirements(drive);
    }

    /* ######################################################################## */
    /* # Standard methods inherited from Command                              # */
    /* ######################################################################## */
    @Override
    public void initialize() {
        m_drive.clearAutoAlignedStatus();
        reloadCommand();
    }

    @Override
    public void execute(){
        m_drive.acceptRequest(
            requestGenerator.apply(
                xSupplier.getAsDouble(),
                ySupplier.getAsDouble(),
                omegaSupplier.getAsDouble()
            )
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
    /* ######################################################################## */
    /* # Internal Utility Methods                                             # */
    /* ######################################################################## */

    /**
     * Reloads the command after a state change. Every time the command's state changes, this function MUST
     * be called for the changes to come into effect
     */
    private void reloadCommand() {
        xSupplier = () -> rawXSupplier.getAsDouble() * transClutch;
        ySupplier = () -> rawYSupplier.getAsDouble() * transClutch;
        omegaSupplier = () -> rawOmegaSupplier.getAsDouble() * rotClutch;
        if (requestGeneratorOverride.isPresent()){
            requestGenerator = requestGeneratorOverride.get();
        } else if (headingSupplier.isPresent()) {
            requestGenerator = (vx, vy, omega) -> {
                return defaultHeadingLockedDriveRequest
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withTargetDirection(headingSupplier.get().get());
            };
        } else {
            requestGenerator = (vx, vy, omega) -> {
                return defaultDriveRequest
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(omega);
            };
        }
    }

    /** Overrides the requestGenerator 
     * Used for custom swerve requests, like x-locking and heading-locking, etc
    */
    private void setRequestGenerator(TriFunction<Double, Double, Double, SwerveRequest> newRequestGenerator){
        this.requestGeneratorOverride = Optional.of(newRequestGenerator);
        reloadCommand();
    }

    /** Clears any requestGenerator overrides and restores default teleop control */
    private void clearRequestGeneratorOverride() {
        this.requestGeneratorOverride = Optional.empty();
        reloadCommand();
    }

    /** Sets an arbitrary clutch factor */
    private void setClutchFactor(double transClutch, double rotClutch) {
        this.transClutch = transClutch;
        this.rotClutch = rotClutch;
        reloadCommand();
    }

    /** Sets an arbitrary heading lock, based on a headingSupplier */
    private void setHeadingLock(Supplier<Rotation2d> headingSupplier) {
        this.headingSupplier = Optional.of(headingSupplier);
        reloadCommand();
    }

    /** Removes any heading lock */
    private void clearHeadingLock() {
        this.headingSupplier = Optional.empty();
        reloadCommand();
    }

    /** Returns whether or not the request generator has been overridden */
    public boolean requestGeneratorOverridden() {
        return requestGeneratorOverride.isPresent();
    }
    
    /* ######################################################################## */
    /* # Public Command Factories                                             # */
    /* ######################################################################## */

    /** Returns a command that applies an arbitrary clutch factor */
    public Command applyClutchFactor(double transClutch, double rotClutch) {
        return Commands.startEnd(
            () -> setClutchFactor(transClutch, rotClutch),
            () -> setClutchFactor(1.0, 1.0)
        );
    }

    /** Returns a command that applies an arbitrary request generator override */
    public Command applyRequestGenerator(TriFunction<Double, Double, Double, SwerveRequest> reqgen) {
        return Commands.startEnd(
            () -> setRequestGenerator(reqgen), 
            () -> clearRequestGeneratorOverride()
        );
    }

    public boolean isAutoAligned() {
        return m_DriveController.atReference();
    }   

    /** Returns a command that makes the drive train drive in chassis-oriented mode, with a clutch applied, for FPV branch alignment */
    public Command applyFPVDrive(){
        return applyRequestGenerator(
            (vx, vy, omega) -> {
                vx *= FPVClutchTranslationFactor;
                vy *= FPVClutchTranslationFactor;
                omega *= FPVClutchRotationFactor;
                return new ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(vx, vy, omega));
            }
        );
    }

    /** Returns a command that applies an arbitrary heading lock, based on a headingSupplier */
    public Command applyHeadingLock(Supplier<Rotation2d> headingSupplier) {
        return Commands.startEnd(
            () -> setHeadingLock(headingSupplier),
            () -> clearHeadingLock()
        );
    }

    /** Returns a command that applies an arbitrary heading lock, based on a static heading */
    public Command applyHeadingLock(Rotation2d heading) {
        return applyHeadingLock(() -> heading);
    }

    /** Returns a command that applies a heading lock such that the front of the robot is always in the
     *  direction of the movement of the robot.
     */
    public Command applySnakeMode() {
        return applyHeadingLock(() -> {
            double x = xSupplier.getAsDouble();
            double y = ySupplier.getAsDouble();
            if ((x * x) + (y * y) > 0.01) {
                return new Rotation2d(x, y);
            } else {
                return Rotation2d.fromRadians(omegaSupplier.getAsDouble());
            }
        });
    }

    /** Returns a command that applies a single clutch to the TeleopDriveCommand */
    public Command applySingleClutch(){
        return applyClutchFactor(singleClutchTranslationFactor, singleClutchRotationFactor);
    }

    /** Returns a command that applies a double clutch to the TeleopDriveCommand */
    public Command applyDoubleClutch(){
        return applyClutchFactor(doubleClutchTranslationFactor, doubleClutchRotationFactor);
    }

    /** Returns a command that applies a forward-oriented heading lock */
    public Command applyForwardHeadingLock() {
        return applyHeadingLock(Rotation2d.fromDegrees(0));
    }

}