// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.SystemConstants.RobotMode;
import frc.robot.PhysicalConstants.VisionConstants;
import frc.robot.PhysicalConstants.VisionConstants.PhotonVision.PhotonConfig;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.Elastic;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureCommandFactory;
import frc.robot.subsystems.climb.climber.ClimberConstants;
import frc.robot.subsystems.climb.climber.ClimberIODisabled;
import frc.robot.subsystems.climb.climber.ClimberIONeo;
import frc.robot.subsystems.climb.climber.ClimberSubsystem;
import frc.robot.subsystems.climb.hook.HookConstants;
import frc.robot.subsystems.climb.hook.HookIODisabled;
import frc.robot.subsystems.climb.hook.HookIONeo;
import frc.robot.subsystems.climb.hook.HookSubsystem;
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
import frc.robot.subsystems.kicker.KickerIODisabled;
import frc.robot.subsystems.kicker.KickerIONeo;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.roller.RollerIODisabled;
import frc.robot.subsystems.roller.RollerIOKraken;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.slapdown.SlapdownConstants;
import frc.robot.subsystems.slapdown.SlapdownIODisabled;
import frc.robot.subsystems.slapdown.SlapdownIOKraken;
import frc.robot.subsystems.slapdown.SlapdownSubsystem;
import frc.robot.subsystems.spindexer.SpindexerIODisabled;
import frc.robot.subsystems.spindexer.SpindexerIOKraken;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretIODisabled;
import frc.robot.subsystems.turret.TurretIOKraken;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.utils.ShiftUtil;
import lib.extendedcommands.MultiToggleableTrigger;
import lib.extendedcommands.ToggleableTrigger;
import lib.hardware.hid.SamuraiPS5Controller;
import lib.hardware.hid.SamuraiXboxController;
import lib.vision.LoggedPhotonVisionLocalizerWithTagPrioritization;
import lib.vision.PhotonVisionLocalizer;
import lib.vision.PhotonVisionLocalizerWithTagPrioritization;
import lib.vision.VisionLocalizationSystem;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.RobotBase;
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

    private final RollerSubsystem m_rollers;
    private final SlapdownSubsystem m_slapdown;
    private final SpindexerSubsystem m_spindexer;
    private final KickerSubsystem m_kicker;

    private final ClimberSubsystem m_climber;
    private final HookSubsystem m_climbHook;

    private final VisionLocalizationSystem m_vision;

    private final Superstructure m_superstructure;

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

        if (SystemConstants.kMode == RobotMode.REAL || RobotBase.isReal()) {
            m_drive = new DriveSubsystem(
                new DriveIOHardware(TunerConstants.createDrivetrain()),
                m_vision,
                m_elastic
            );
            m_turret = new TurretSubsystem(new TurretIOKraken());
            m_flywheel = new FlywheelSubsystem(new FlywheelIOKraken());
            m_hood = new HoodSubsystem(new HoodIONeo());

            m_rollers = new RollerSubsystem(new RollerIOKraken());
            m_slapdown = new SlapdownSubsystem(new SlapdownIOKraken());
            m_spindexer = new SpindexerSubsystem(new SpindexerIOKraken());
            m_kicker = new KickerSubsystem(new KickerIONeo());

            m_climber = new ClimberSubsystem(new ClimberIONeo());
            m_climbHook = new HookSubsystem(new HookIONeo());

            for (PhotonConfig config : PhotonConfig.values()) {
                PhotonCamera cam = new PhotonCamera(config.name);
                m_vision.addCamera(new PhotonVisionLocalizerWithTagPrioritization(
                    cam,
                    config.offset,
                    config.multiTagPoseStrategy,
                    config.singleTagPoseStrategy,
                    () -> m_drive.getPose().getRotation(),
                    PhysicalConstants.VisionConstants.kAprilTagFieldLayout,
                    config.defaultSingleTagStdDevs.times(2),
                    config.defaultMultiTagStdDevs.times(2),
                    VisionConstants.kHubTags,
                    0.5)
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

            m_rollers = new RollerSubsystem(new RollerIODisabled());
            m_slapdown = new SlapdownSubsystem(new SlapdownIODisabled());
            m_spindexer = new SpindexerSubsystem(new SpindexerIODisabled());
            m_kicker = new KickerSubsystem(new KickerIODisabled());

            m_climber = new ClimberSubsystem(new ClimberIODisabled());
            m_climbHook = new HookSubsystem(new HookIODisabled());
        }

        teleopDriveCommand = new TeleopDriveCommand(
            m_drive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()
        );

        m_superstructure = new Superstructure(
            m_turret,
            m_flywheel,
            m_hood,
            m_rollers,
            m_slapdown,
            m_spindexer,
            m_kicker,
            m_climber,
            m_climbHook,
            () -> m_drive.getPose(),
            () -> m_drive.getChassisSpeeds()
        );

        // Configure the trigger bindings
        configureBindings();

        configureDriverBindings();

        configureOperatorBindings();

        m_superstructure.configureStateBasedBindings();
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
        new Trigger(() -> m_elastic.readNumber("FlywheelVelocityTarget") > 50)
            .onTrue(m_superstructure.getCommandFactory().applyTurretManualState())
            .whileTrue(m_flywheel.runVelocity(() -> m_elastic.readNumber("FlywheelVelocityTarget")));
    }

    /** Bind triggers on driver controller to commands */
    private void configureDriverBindings() {
        SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandFactory();

        m_drive.setDefaultCommand(teleopDriveCommand);

        m_driverController.L1().and(m_driverController.R1().negate())
            .whileTrue(teleopDriveCommand.applyDoubleClutch());

        m_driverController.R1().and(m_driverController.L1().negate())
            .whileTrue(teleopDriveCommand.applySingleClutch());

        m_driverController.create()
            .onTrue(Commands.runOnce(() ->
                m_drive.resetHeading()
            ));

        new ToggleableTrigger(m_driverController.touchpad(), false).getToggledTrigger()
            .whileTrue(teleopDriveCommand.applySnakeMode());

        m_driverController.L2()
            .onTrue(superstructureCommands.intake())
            .onFalse(superstructureCommands.applyIntakeExtended());

        m_driverController.R2()
            .onTrue(superstructureCommands.shoot())
            .onFalse(superstructureCommands.stopShooting());

        m_driverController.square()
            .onTrue(superstructureCommands.applyIntakeRetracted());

        m_driverController.cross()
            .onTrue(superstructureCommands.applyTurretStatesHubPreAlignedLocation());

            /*
        m_driverController.povUp()
            .onTrue(m_hood.applyVoltageUnrestricted(4))
            .onFalse(m_hood.applyVoltageUnrestricted(0));

        m_driverController.povDown()
            .onTrue(m_hood.applyVoltageUnrestricted(-4))
            .onFalse(m_hood.applyVoltageUnrestricted(0)); 

        m_driverController.povLeft()
            .onTrue(m_climbHook.applyVoltage(4))
            .onFalse(m_climbHook.applyVoltage(0));

        m_driverController.povRight()
            .onTrue(m_climbHook.applyVoltage(-4))
            .onFalse(m_climbHook.applyVoltage(0)); */

        // Trench mode!
        new ToggleableTrigger(m_driverController.circle(), false).getToggledTrigger()
            .onTrue(superstructureCommands.applyTurretIdle())
            .onFalse(superstructureCommands.initiateAutoaim());

        m_driverController.povUp()
            .onTrue(m_hood.applyPosition(0.1));

        m_driverController.povDown()
            .onTrue(m_hood.applyPosition(0.3));
        
        m_driverController.povLeft()
            .onTrue(m_turret.applyPosition(0));

        m_driverController.povRight()
            .onTrue(m_turret.applyPosition(2));

        m_driverController.triangle()
            .onTrue(superstructureCommands.initiateAutoaim());
    }

    /** Bind triggers on operator controller to commands */
    private void configureOperatorBindings() {
        SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandFactory();

        // The A, B, and Y buttons are used to toggle between controlling the turret/hood, slapdown, and climber with the joysticks.
        MultiToggleableTrigger joystickTriggers = new MultiToggleableTrigger(m_operatorController.a(), m_operatorController.b(), m_operatorController.y());

        m_operatorController.leftActive().and(joystickTriggers.getToggledTrigger(0))
            .whileTrue(
                Commands.sequence(
                    superstructureCommands.applyTurretManualState(),
                    m_turret.runVoltageUnrestricted(
                        () -> -m_operatorController.getLeftX() * TurretConstants.kMaxManualControlVolts)
                )
            )
            .onFalse(m_turret.applyVoltageUnrestricted(0));

        m_operatorController.rightActive().and(joystickTriggers.getToggledTrigger(0))
            .whileTrue(Commands.parallel(
                    superstructureCommands.applyTurretManualState(),
                    m_hood.runVoltageUnrestricted(
                        () -> -m_operatorController.getRightY() * HoodConstants.kMaxOperatorControlVolts)
                ))
            .onFalse(m_hood.applyVoltageUnrestricted(0)); 

        m_operatorController.leftActive().and(joystickTriggers.getToggledTrigger(1))
            .whileTrue(m_slapdown.runVoltageUnrestricted(
                () -> -m_operatorController.getLeftX() * SlapdownConstants.kMaxOperatorControlVolts))
            .onFalse(m_slapdown.applyVoltageUnrestricted(0));

        m_operatorController.leftActive().and(joystickTriggers.getToggledTrigger(2))
            .whileTrue(m_climber.runVoltage(
                () -> -m_operatorController.getLeftY() * ClimberConstants.kMaxOperatorControlVolts))
            .onFalse(m_climber.applyVoltage(0));
        
        m_operatorController.rightActive().and(joystickTriggers.getToggledTrigger(2))
            .whileTrue(m_climbHook.runVoltage(
                () -> -m_operatorController.getRightX() * HookConstants.kMaxOperatorControlVolts))
            .onFalse(m_climbHook.applyVoltage(0));
        
        m_operatorController.povUp()
            .onTrue(Commands.sequence(
                superstructureCommands.startTurretManualControl(),
                m_flywheel.applyVelocity(500))
            );
        
        m_operatorController.povLeft()
            .onTrue(Commands.sequence(
                superstructureCommands.startTurretManualControl(),
                m_flywheel.decrementVelocity())
            );

        m_operatorController.povRight()
            .onTrue(Commands.sequence(
                superstructureCommands.startTurretManualControl(),
                m_flywheel.incrementVelocity())
            );
        
        m_operatorController.povDown()
            .onTrue(Commands.sequence(
                superstructureCommands.startTurretManualControl(),
                m_flywheel.applyVelocity(0))
            );
        
        // Sets the rollers to do the opposite of their current state
        m_operatorController.leftTrigger()
            .onTrue(Commands.either(
                m_rollers.applyVoltage(6),
                m_rollers.applyVoltage(0),
                () -> m_rollers.getVoltage() < 0.5
            ));

        // Turns the spindexer to the opposite of it's current state
        m_operatorController.leftBumper()
            .onTrue(Commands.either(
                m_kicker.applyVoltage(12),
                m_kicker.applyVoltage(0),
                () -> m_kicker.getVoltage() < 0.5
            ));
        
        // Turns the spindexer to the opposite of it's current state    
        m_operatorController.rightBumper()
            .onTrue(Commands.either(
                m_spindexer.applyVoltage(12),
                m_spindexer.applyVoltage(0),
                () -> m_spindexer.getVoltage() < 0.5
        ));
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

    /** Returns the amount of time left in the shift */
    @AutoLogOutput(key = "Match/ShiftTime", unit="second")
    public int getShiftTime() {
        return ShiftUtil.getSecondsRemainingInShift();
    }

    /** Returns if the our hub is active */
    @AutoLogOutput(key = "Match/OurHubActive")
    public boolean isOurHubActive() {
        return ShiftUtil.isOurAllianceActive();
    }

    /** Returns if blue alliance won auto */
    @AutoLogOutput(key = "Match/Auton Winner Color Hex")
    public String getAutonWinnerColorHex() {
        return ShiftUtil.autonWinnerColorHex();
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
