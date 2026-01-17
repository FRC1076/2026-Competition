// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.SystemConstants.RobotMode;
import frc.robot.FieldConstants.AprilTagLayoutType;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIONorthstar;
import lib.hardware.hid.SamuraiPS5Controller;
import lib.hardware.hid.SamuraiXboxController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
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
    private final TurretSubsystem m_turret;
    private final FlywheelSubsystem m_flywheel;
    private final HoodSubsystem m_hood;
    private final Vision m_vision;

    // Controllers
    private final SamuraiPS5Controller m_driverController =
        new SamuraiPS5Controller(OIConstants.kDriverControllerPort);
    private final SamuraiXboxController m_operatorController = 
        new SamuraiXboxController(OIConstants.kOperatorControllerPort);
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        if (SystemConstants.kMode == RobotMode.REAL) {
            m_turret = new TurretSubsystem(new TurretIOKraken());
            m_flywheel = new FlywheelSubsystem(new FlywheelIOKraken());
            m_hood = new HoodSubsystem(new HoodIONeo());
        } else {
            m_turret = new TurretSubsystem(new TurretIODisabled());
            m_flywheel = new FlywheelSubsystem(new FlywheelIODisabled());
            m_hood = new HoodSubsystem(new HoodIODisabled());
        }

        AprilTagLayoutType type = FieldConstants.defaultAprilTagType;
        m_vision = new Vision(type, 
            new VisionIONorthstar(type, 0));

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
        m_driverController.getLeftX(); // swerve stuff goes here when that is done
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
