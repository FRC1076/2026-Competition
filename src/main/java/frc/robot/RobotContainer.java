// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.SystemConstants.RobotMode;
import frc.robot.Constants.VisionConstants.PhotonVision.PhotonConfig;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.Elastic;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.flywheel.FlywheelIODisabled;
import frc.robot.subsystems.flywheel.FlywheelIOKraken;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodIODisabled;
import frc.robot.subsystems.hood.HoodIONeo;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretIODisabled;
import frc.robot.subsystems.turret.TurretIOKraken;
import frc.robot.subsystems.turret.TurretSubsystem;
import lib.hardware.hid.SamuraiPS5Controller;
import lib.hardware.hid.SamuraiXboxController;
import lib.vision.PhotonVisionLocalizer;
import lib.vision.VisionLocalizationSystem;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_drive;
    private final TurretSubsystem m_turret;
    private final FlywheelSubsystem m_flywheel;
    private final HoodSubsystem m_hood;
    private final VisionLocalizationSystem m_vision;

    private final Elastic m_elastic;

    // Controllers
    private final SamuraiPS5Controller m_driverController =
        new SamuraiPS5Controller(OIConstants.kDriverControllerPort);
    private final SamuraiXboxController m_operatorController = 
        new SamuraiXboxController(OIConstants.kOperatorControllerPort);

    TeleopDriveCommand teleopDriveCommand;
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Vision and elastic are independent of being real or simulated
        m_vision = new VisionLocalizationSystem();
        m_elastic = new Elastic();

        if (SystemConstants.kMode == RobotMode.REAL) {
            m_drive = new DriveSubsystem(
                new DriveIOHardware(TunerConstants.createDrivetrain()),
                m_vision,
                m_elastic
            );
            m_turret = new TurretSubsystem(new TurretIOKraken());
            m_flywheel = new FlywheelSubsystem(new FlywheelIOKraken());
            m_hood = new HoodSubsystem(new HoodIONeo());

            for (PhotonConfig config : PhotonConfig.values()) {
                PhotonCamera cam = new PhotonCamera(config.name);
                m_vision.addCamera(new PhotonVisionLocalizer(
                    cam,
                    config.offset,
                    config.multiTagPoseStrategy,
                    config.singleTagPoseStrategy,
                    () -> m_drive.getPose().getRotation(),
                    VisionConstants.kAprilTagFieldLayout,
                    config.defaultSingleTagStdDevs,
                    config.defaultMultiTagStdDevs)
                );
            }
        } else {
            m_drive = new DriveSubsystem(
                new DriveIOSim(TunerConstants.createDrivetrain()),
                m_vision,
                m_elastic
            );
            m_turret = new TurretSubsystem(new TurretIODisabled());
            m_flywheel = new FlywheelSubsystem(new FlywheelIODisabled());
            m_hood = new HoodSubsystem(new HoodIODisabled());
        }

        teleopDriveCommand = new TeleopDriveCommand(
            m_drive,
            () -> m_driverController.getLeftX(),
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getRightX()
        );

        // Configure the trigger bindings
        configureBindings();

        configureDriverBindings();

        configureOperatorBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

    }

    /** Bind triggers on driver controller to commands */
    private void configureDriverBindings() {
        m_drive.setDefaultCommand(teleopDriveCommand);
    }

    /** Bind triggers on operator controller to commands */
    private void configureOperatorBindings() {
        m_operatorController.leftActive()
            .whileTrue(m_turret.applyVoltage(
                    m_operatorController.getLeftX() * TurretConstants.kMaxManualControlVolts));

        m_operatorController.rightActive()
            .whileTrue(m_hood.applyVoltage(
                m_operatorController.getRightY() * HoodConstants.kMaxOperatorControlVolts));

        m_operatorController.a()
            .onTrue(m_flywheel.applyVoltage(0));
        
        m_operatorController.b()
            .onTrue(m_flywheel.applyVoltage(4));

        m_operatorController.x()
            .onTrue(m_flywheel.applyVoltage(8));

        m_operatorController.y()
            .onTrue(m_flywheel.applyVoltage(12));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Commands.none();
    }

    /** Command to raise thread priority */
    public static Command threadCommand() {
        return Commands.sequence(
            Commands.waitSeconds(20),
            Commands.runOnce(() -> Threads.setCurrentThreadPriority(true, 1)),
            Commands.print("Main Thread Priority raised to RT1 at " + Timer.getFPGATimestamp())
        ).ignoringDisable(true);
    }
}
