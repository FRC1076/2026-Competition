// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.PhysicalConstants;
import frc.robot.subsystems.SuperstructureConstants.ClimbStates;
import frc.robot.subsystems.SuperstructureConstants.FieldTargets;
import frc.robot.subsystems.SuperstructureConstants.IndexStates;
import frc.robot.subsystems.SuperstructureConstants.IntakeStates;
import frc.robot.subsystems.SuperstructureConstants.TurretStates;
import frc.robot.subsystems.climb.climber.ClimberSubsystem;
import frc.robot.subsystems.climb.hook.HookSubsystem;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.slapdown.SlapdownSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import lib.ballistic.BallCounter;
import lib.ballistic.CommonShotSolution;
import lib.ballistic.SOTMLaunchCalculator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Superstructure {
    public class MutableSuperState{
        private TurretStates turretState = TurretStates.IDLE;
        private IntakeStates intake = IntakeStates.RETRACTED;
        private IndexStates index = IndexStates.IDLE;
        private ClimbStates climb = ClimbStates.IDLE;
        private boolean turretRotationEnabled = true;
        
        /** Returns the state of the turret assembly */
        public TurretStates getTurretState() {
            return turretState;
        }

        /** Returns the state of the turret assembly as a string */
        @AutoLogOutput(key = "SuperState/TurretState")
        public String getTurretStateString() {
            return turretState.toString();
        }
        
        /** Sets the target state of the turret assembly
         *  (note: this does not actually affect the physical robot, it only
         *   affects the variable in the code)
         */
        public void setTurretState(TurretStates newTurretState) {
            turretState = newTurretState;
        }

        /** Returns the index state. */
        public IndexStates getIndexState() {
            return index;
        }

        /** Returns the index state as a string */
        @AutoLogOutput(key = "SuperState/IndexState")
        public String getIndexStateString() {
            return index.toString();
        }

        /** Returns the previous fuel management state of the robot */

        /** Sets the Index state
         *  (note: this does not actually affect the physical robot, it only
         *   affects the variable in the code)
          */
        public void setIndexState(IndexStates newIndexState) {
            index = newIndexState;
        }

        /** Returns the intake state. */
        public IntakeStates getIntakeState() {
            return intake;
        }

        /** Returns whether or not the turret is on */
        @AutoLogOutput(key = "SuperState/turretRotationEnabled")
        public boolean getturretRotationEnabled() {
            return turretRotationEnabled;
        }

        /** Returns the intake state as a string */
        @AutoLogOutput(key = "SuperState/IntakeState")
        public String getIntakeStateString() {
            return intake.toString();
        }

        /** Sets the Intake state
         *  (note: this does not actually affect the physical robot, it only
         *   affects the variable in the code)
          */
        public void setIntakeState(IntakeStates newIntakeState) {
            intake = newIntakeState;
        }

        /** Returns the current commanded climb state. */
        public ClimbStates getClimbState() {
            return climb;
        }

        /** Sets the Climb state
         *  (note: this does not actually affect the physical robot, it only
         *   affects the variable in the code)
         */
        public void setClimbState(ClimbStates newClimbState) {
            climb = newClimbState;
        }

        /** Sets whether or not the turret should be run */
        public void setEnableTurretRotation(boolean state) {
            turretRotationEnabled = state;
        }

        /** Default constructor. Sets all variables to IDLE */
        public MutableSuperState() {
            this.turretState = TurretStates.IDLE;
            this.index = IndexStates.IDLE;
        }
        
        /** Instantiates a MutableSuperState with the specified TurretState and FuelManagementeState */
        public MutableSuperState(TurretStates turretState, IndexStates index) {
            this.turretState = turretState;
            this.index = index;
        }
    }
    
    private final TurretSubsystem m_turret;
    private final FlywheelSubsystem m_flywheel;
    private final HoodSubsystem m_hood;

    private final RollerSubsystem m_roller;
    private final SlapdownSubsystem m_slapdown;
    private final SpindexerSubsystem m_spindexer;
    private final KickerSubsystem m_kicker;

    private final ClimberSubsystem m_climber;
    private final HookSubsystem m_climbHook;

    private final Supplier<Pose2d> m_robotPoseSupplier;
    private final Supplier<ChassisSpeeds> m_robotVelocitySupplier;
    private final BooleanSupplier m_passInAutoSupplier;

    private final MutableSuperState m_superState;
    private final SuperstructureCommandFactory m_commandFactory;

    private CommonShotSolution m_shootingParams;
    private FieldTargets fieldTargets = SuperstructureConstants.kBlueAllianceTargets;
    private boolean targetIsHub = true;

    private final BallCounter ballCounter;

    public Superstructure(
        TurretSubsystem turret,
        FlywheelSubsystem flywheel,
        HoodSubsystem hood,
        RollerSubsystem roller,
        SlapdownSubsystem slapdown,
        SpindexerSubsystem spindexer,
        KickerSubsystem kicker,
        ClimberSubsystem climber,
        HookSubsystem climbHook,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisSpeeds> robotVelocitySupplier,
        BooleanSupplier passInAutoSupplier
    ) {
        this.m_turret = turret;
        this.m_flywheel = flywheel;
        this.m_hood = hood;
        this.m_roller = roller;
        this.m_slapdown = slapdown;
        this.m_spindexer = spindexer;
        this.m_kicker = kicker;
        this.m_climber = climber;
        this.m_climbHook = climbHook;

        this.m_robotPoseSupplier = robotPoseSupplier; 
        this.m_robotVelocitySupplier = robotVelocitySupplier;

        this.m_superState = new MutableSuperState();

        this.m_commandFactory = new SuperstructureCommandFactory(this);

        m_shootingParams = new CommonShotSolution(0, 0, 0);
    
        this.ballCounter = new BallCounter(
            () -> m_flywheel.getVelocityRadPerSec(),
            () -> m_flywheel.getPIDTargetRadPerSec(), 
            SuperstructureConstants.kFlywheelVelocityDropRatio,
            SuperstructureConstants.kFlywheelVelocityRecoveryRatio, 
            () -> targetIsHub
        );

        this.m_passInAutoSupplier = passInAutoSupplier;
    }

    /** Get the Command factory */
    public SuperstructureCommandFactory getCommandFactory() {
        return m_commandFactory;
    }

    /** Configure trigger bindings that are based upon the state of the robot. */
    public void configureStateBasedBindings() {
        new Trigger(() -> m_superState.getTurretState().kIsAutoAim)
            .whileTrue(
                Commands.parallel(
                    Commands.run(() -> updateShootingParams()),
                    m_flywheel.runVelocity(() -> m_shootingParams.launchSpeedRadPerSec()),
                    m_hood.runPosition(() -> m_shootingParams.launchPitchRad()),
                    m_turret.runPositionSafe(() -> m_shootingParams.launchYawRad(), () -> m_superState.getturretRotationEnabled())
                )
            );

        Trigger runSlapdownShake = new Trigger(() -> m_superState.getIntakeState().kRunSlapdownShake);
        runSlapdownShake
            .whileTrue(
                m_slapdown.runPosition(
                    () -> (Timer.getFPGATimestamp() % m_superState.getIntakeState().kSlapdownShakePeriodSecs
                        < m_superState.getIntakeState().kSlapdownShakePeriodSecs / 2.0)
                            ? m_superState.getIntakeState().kSlapdownAngle : m_superState.getIntakeState().kSlapdownShakeUpAngle
                )
            );
            
        Trigger isAutonomous = new Trigger(() -> DriverStation.isAutonomousEnabled());
        Trigger autonRunSlapdown = isAutonomous.and(runSlapdownShake.negate());
        Trigger slapdownInPosition = new Trigger(() -> m_slapdown.withinTolerance(() -> m_superState.getIntakeState().kSlapdownAngle));
        autonRunSlapdown.and(slapdownInPosition.negate())
            .whileTrue(m_slapdown.runPosition(() -> m_superState.getIntakeState().kSlapdownAngle));
        autonRunSlapdown.and(slapdownInPosition)
            .whileTrue(m_slapdown.runHoldPositionStrong(() -> m_superState.getIntakeState().kSlapdownAngle));

        isAutonomous.onTrue(Commands.runOnce(() -> ballCounter.resetCount()));

        new Trigger(() -> DriverStation.isEnabled())
            .whileTrue(Commands.run(() -> ballCounter.update()));
    }

    /** Update parameters saved to m_shootingParams and flywheelTargetRadiansPerSecond
     *  based upon the current pose of the robot.
     */
    private void updateShootingParams() {
        final Pose3d shooterPose = new Pose3d(m_robotPoseSupplier.get()).transformBy(PhysicalConstants.kBotRelativeTurretPose);
        final ChassisSpeeds robotVelocity = m_robotVelocitySupplier.get();
        
        final Pose3d shooterPoseAllianceColorCoordinates = shooterPose.relativeTo(fieldTargets.kAllianceOrigin());

        // Translate the chassis speeds
        final ChassisSpeeds turretVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(
            robotVelocity.vxMetersPerSecond - (robotVelocity.omegaRadiansPerSecond * PhysicalConstants.kBotRelativeTurretPose.getY()),
            robotVelocity.vyMetersPerSecond + (robotVelocity.omegaRadiansPerSecond * PhysicalConstants.kBotRelativeTurretPose.getX()),
            robotVelocity.omegaRadiansPerSecond
        ), shooterPose.getRotation().toRotation2d());
        
        targetIsHub = true;
        // TODO: check tolerance (+ 0.25) on alliance zone (otherwise we'll ram into the trench)
        if (shooterPoseAllianceColorCoordinates.getX() <= PhysicalConstants.FieldConstants.LinesVertical.allianceZone + 0.2 || (DriverStation.isAutonomous() && !m_passInAutoSupplier.getAsBoolean())) {
            m_shootingParams = SOTMLaunchCalculator.calculateHub(shooterPose.toPose2d(), fieldTargets.kHubTarget().toPose2d(), turretVelocity);
            return;
        } else if (shooterPoseAllianceColorCoordinates.getX() <= PhysicalConstants.FieldConstants.LinesVertical.neutralZoneNear
                && shooterPoseAllianceColorCoordinates.getX() >= PhysicalConstants.FieldConstants.LinesVertical.allianceZone) {
            m_shootingParams = SOTMLaunchCalculator.calculateHub(shooterPose.toPose2d(), fieldTargets.kHubTarget().toPose2d(), turretVelocity);
            m_shootingParams = CommonShotSolution.withZeroPitch(m_shootingParams); // Don't ram into the trench
            return;
        }

        // Not in the alliance zone, resort to passing
        Pose2d passingTarget =
            shooterPoseAllianceColorCoordinates.getY() <= PhysicalConstants.FieldConstants.LinesHorizontal.center // Are we on right side?
                ? fieldTargets.kRightPassingTarget().toPose2d() // Yes
                : fieldTargets.kLeftPassingTarget().toPose2d(); // No
        targetIsHub = false;
        
        if (shooterPoseAllianceColorCoordinates.getX() >= PhysicalConstants.FieldConstants.LinesVertical.neutralZoneFar
                && shooterPoseAllianceColorCoordinates.getX() <= PhysicalConstants.FieldConstants.LinesVertical.oppAllianceZone) {
            // Zero out pitch so we don't ram into the trench
            m_shootingParams = CommonShotSolution.withZeroPitch(SOTMLaunchCalculator.calculatePass(shooterPose.toPose2d(), passingTarget, turretVelocity));
            return;
        } else {
            m_shootingParams = SOTMLaunchCalculator.calculatePass(shooterPose.toPose2d(), passingTarget, turretVelocity);
            return;
        }
    }

    public MutableSuperState getSuperState() {
        return m_superState;
    }

    /** Sets the team color for where to shoot fuel */
    public void setTeamColor(Alliance color) {
        if (color == Alliance.Blue) {
            fieldTargets = SuperstructureConstants.kBlueAllianceTargets;
        } else {
            fieldTargets = SuperstructureConstants.kRedAllianceTargets;
        }
    }

    /** Returns a trigger that is true when the robot is ready to shoot fuel */
    public Trigger isReadyToShoot() {
        return new Trigger(() ->
            m_flywheel.getVelocityRadPerSec() > SuperstructureConstants.kMinFlywheelShootingVelocity && 
            (!m_superState.getTurretState().kIsAutoAim ||
                //(m_flywheel.readyToShoot(m_shootingParams.launchSpeedRadPerSec()) 
                m_flywheel.getVelocityRadPerSec() > SuperstructureConstants.kMinFlywheelShootingVelocity
                && m_turret.withinTolerance(m_shootingParams.launchYawRad()))
        );
    }

    /** Returns whether or not the slapdown is too high to move the turret */
    public BooleanSupplier safeToSpinRollers() {
        return () -> m_slapdown.getSlapdownAngleRadians() > SuperstructureConstants.kTurretMoveSlapdownAngleLimitRad; // For some reason down is larger number
    }

    public Trigger isDoneShooting() {
        return m_flywheel.hasStoppedShooting(() -> m_shootingParams.launchSpeedRadPerSec());
    }

    /** Applies the target state to the rollers when safe */
    public Command applyRollerVoltageUponSafe(double voltage) {
        return Commands.sequence(
            Commands.waitUntil(safeToSpinRollers()),
            m_roller.applyVoltage(voltage)
        );
    }

    /** Sets the intake state in the Super State. Does not affect the physical robot. */
    public Command setIntakeState(IntakeStates state) {
        return Commands.runOnce(() -> m_superState.setIntakeState(state));
    }

    /** Sets the index state in the Super State. Does not affect the physical robot. */
    public Command setIndexState(IndexStates state) {
        return Commands.runOnce(() -> m_superState.setIndexState(state));
    }

    /** Sets the turret state to the desired item such so that a state-based trigger can take control */
    public Command setTurretState(TurretStates state) {
        return Commands.runOnce(() -> m_superState.setTurretState(state));
    }

    /** Sets the climb state to the desired target in Super State. Does not affect the physical robot. */
    public Command setClimbState(ClimbStates state) {
        return Commands.runOnce(() -> m_superState.setClimbState(state));
    }

    /** Sets the state of whether or not to run the turret for auto-aim */
    public Command setTurretRotationEnabledState(boolean state) {
        return Commands.runOnce(() -> m_superState.setEnableTurretRotation(state));
    }

    public Command applyIntakeStateRollerOnly(IntakeStates state) {
        return Commands.parallel(
            setIntakeState(state),
            applyRollerVoltageUponSafe(state.kRollerVoltage)
        );
    }

    public Command applyIntakeStateAllParallel(IntakeStates state) {
        /*
        return Commands.parallel(
            setIntakeState(state),
            m_slapdown.applyPosition(state.kSlapdownAngle),
            applyRollerVoltageUponSafe(state.kRollerVoltage)
        ); */

        return Commands.sequence(
            Commands.parallel(
                setIntakeState(state),
                m_slapdown.applyPosition(state.kSlapdownAngle),
                applyRollerVoltageUponSafe(state.kRollerVoltage)
            ),
            Commands.waitUntil(() -> m_slapdown.withinTolerance(state.kSlapdownAngle)),
            m_slapdown.holdPositionStrong(state.kSlapdownAngle)
        );
    }

    public Command applyIndexStateAllParallel(IndexStates state) {
        return Commands.parallel(
            setIndexState(state),
            m_spindexer.applyVelocity(state.kSpindexerVelocity),
            m_kicker.applyVelocity(state.kKickerVelocity)
        );
    }

    public Command applyIndexStateSpindexerOnly(IndexStates state) {
        return m_spindexer.applyVelocity(state.kSpindexerVelocity);
    }

    /** Apply the passed in state to the turret, hood, and flywheel, all at the same time. */
    public Command applyTurretStateAllParallel(TurretStates state) {
        return Commands.parallel(
            setTurretState(state),
            m_turret.applyPosition(state.kTurretAngleRadians),
            m_flywheel.applyVelocity(state.kFlywheelRadPerSec),
            m_hood.applyPosition(state.kHoodAngleRadians)
        );
    }

    /** Apply the passed in state to the turret only. */
    public Command applyTurretStateFlywheelOnly(TurretStates state) {
        return Commands.parallel(
            setTurretState(state),
            m_flywheel.applyVelocity(state.kFlywheelRadPerSec)
        );
    }

    /** Apply the passed in state to the climber and climb hook */
    public Command applyClimbStateAllParallel(ClimbStates state) {
        return Commands.parallel(
            setClimbState(state),
            m_climber.applyPosition(state.kClimberPosition),
            m_climbHook.applyPosition(state.kHookPosition)
        );
    }

    /** Public Commands to be used to bind to triggers in RobotContainer */
    public class SuperstructureCommandFactory{
        private final Superstructure m_superstructure;

        public SuperstructureCommandFactory(Superstructure superstructure){
            this.m_superstructure = superstructure;
        }

        /** Apply the IDLE turret state. */
        public Command applyTurretIdle(){
            return m_superstructure.applyTurretStateAllParallel(TurretStates.IDLE);
        }

        /** Apply the MANUAL turret state */
        public Command applyTurretManualState(){
            return m_superstructure.setTurretState(TurretStates.MANUAL);
        }

        /** Apply the ATOUAIM_IDLE turret state (gets ready to shoot) */
        public Command initiateAutoaim(){
            return m_superstructure.setTurretState(TurretStates.AUTOAIM_IDLE);
        }

        /** Max out flywheel speed for shooting */
        public Command shootWithAutoaim() {
            return m_superstructure.setTurretState(TurretStates.AUTOAIM_SHOOT);
        }

        /** Allow for manual control */
        public Command startTurretManualControl() {
            return m_superstructure.setTurretState(TurretStates.MANUAL);
        }

        /** Apply the HUB_PREALIGNED_LOCATION turret state */
        public Command applyTurretStatesHubPreAlignedLocation(){
            return m_superstructure.applyTurretStateAllParallel(TurretStates.HUB_PREALIGNED_LOCATION);
        }

        public Command applyTurretStatesLeftTrenchPrealigned() {
            return m_superstructure.applyTurretStateAllParallel(TurretStates.TRENCH_PREALIGNED_LEFT);
        }

        public Command applyTurretStatesRightTrenchPrealigned() {
            return m_superstructure.applyTurretStateAllParallel(TurretStates.TRENCH_PREALIGNED_RIGHT);
        }

        /** Apply the POINT_DIRECTLY_BACK_FOR_PASSING turret state */
        public Command applyTurretStatesPointDirectlyBackForPassing(){
            return m_superstructure.applyTurretStateAllParallel(TurretStates.POINT_DIRECTLY_BACK_FOR_PASSING);
        }

        /** Apply the RETRACTED state for the intake */
        public Command applyIntakeRetracted() {
            return m_superstructure.applyIntakeStateAllParallel(IntakeStates.RETRACTED);
        }

        /** Apply the EXTENDED state for the intake */
        public Command applyIntakeExtended() {
            return m_superstructure.applyIntakeStateAllParallel(IntakeStates.EXTENDED);
        }

        /** Start intaking fuel */
        public Command intake() {
            return m_superstructure.applyIntakeStateAllParallel(IntakeStates.INTAKING);
        }

        /** Intake fuel only by running the rollers (for autonomous weirdness) */
        public Command runRollers() {
            return m_superstructure.applyIntakeStateRollerOnly(IntakeStates.INTAKING);
        }

        /** Index fuel for shooting */
        public Command startIndexing() {
            return Commands.sequence(
                m_superstructure.setIndexState(IndexStates.INDEXING),
                m_spindexer.applyVelocity(() -> m_superState.getIndexState().kSpindexerVelocity),
                Commands.race(
                    Commands.waitUntil(() -> m_spindexer.getVelocityRadPerSec() > SuperstructureConstants.kMinSpindexerIndexerVelocity),
                    Commands.waitSeconds(0.5)
                ),
                m_kicker.applyVelocity(() -> m_superState.getIndexState().kKickerVelocity)
            );
            
            //m_superstructure.applyIndexStateAllParallel(IndexStates.INDEXING);
        }

        /** Set the spindexer and kicker to idle */
        public Command stopIndexing() {
            return m_superstructure.applyIndexStateAllParallel(IndexStates.IDLE);
        }

        public Command reverseEverything() {
            return Commands.parallel(
                m_superstructure.applyTurretStateFlywheelOnly(TurretStates.REVERSE),
                m_superstructure.applyIndexStateAllParallel(IndexStates.REVERSE),
                m_superstructure.applyIntakeStateRollerOnly(IntakeStates.REVERSE)
            );
        }

        public Command stopReverseEverything() {
            return Commands.parallel(
                m_superstructure.applyTurretStateFlywheelOnly(TurretStates.IDLE),
                m_superstructure.applyIndexStateAllParallel(IndexStates.IDLE),
                m_superstructure.applyIntakeStateRollerOnly(IntakeStates.EXTENDED)
            );
        }

        public Command reverseSpindexer() {
            return m_superstructure.applyIndexStateSpindexerOnly(IndexStates.REVERSE);
        }

        public Command stopReverseSpindexer() {
            return m_superstructure.applyIndexStateSpindexerOnly(IndexStates.IDLE);
        }

        public Command intakeAndIndexToShoot() {
            return startIndexing();

            /*
            return Commands.parallel(
                startIndexing(),
                applyIntakeStateRollerOnly(IntakeStates.SHOOTING)
            ); */
        }

        public Command stopIntakeAndIndexToShoot() {
            return stopIndexing();

            /*
            return Commands.parallel(
                stopIndexing(),
                applyIntakeStateRollerOnly(IntakeStates.EXTENDED)
            );
            */
        }


        /** Shoot fuel. Either goes to AUTOAIM_SHOOT for turret,
         *  or does nothing for turret depending on if AUTOAIM is active.  */
        public Command shoot() {
            return Commands.either(
                Commands.sequence(
                    shootWithAutoaim(),
                    intakeAndIndexToShoot()
                ),
                Commands.sequence(
                    intakeAndIndexToShoot()
                ),
                () -> m_superstructure.getSuperState().getTurretState().kIsAutoAim
            );
        }

        /** Stop shooting fuel. Either goes to AUTOAIM_IDLE for turret or does nothing for turret,
         *  or does nothing for turret, depending on if AUTOAIM is active.
         */
        public Command stopShooting() {
            return Commands.either(
                Commands.sequence(
                    stopIntakeAndIndexToShoot(),
                    initiateAutoaim()
                ), 
                stopIntakeAndIndexToShoot(), 
                () -> m_superstructure.getSuperState().getTurretState().kIsAutoAim
            );
        }

        public Command waitUntilDoneShooting() {
            return Commands.waitUntil(isDoneShooting());
        }

        /** Start kicking the intake */
        public Command startSlapdownShake() {
            return applyIntakeStateRollerOnly(IntakeStates.RUN_KICK);
        }

        public Command startSlapdownShakeAutonomous() {
            return applyIntakeStateRollerOnly(IntakeStates.RUN_KICK);
        }

        public Command stopSlapdownShakeAutonomous() {
            return applyIntakeStateRollerOnly(IntakeStates.INTAKING);
        }

        /** Stop kicking the intake */
        public Command stopSlapdownShake(Trigger continueRollersSelector) {
            return Commands.either(
                intake(),
                applyIntakeExtended(),
                continueRollersSelector
            );
        }

        /** Retracts the climber and climb hook */
        public Command applyClimbIdle() {
            return applyClimbStateAllParallel(ClimbStates.IDLE);
        }

        /** Pulls up the intake and extends the climber to get ready to climb */
        public Command getReadyToClimb() {
            return Commands.sequence(
                applyIntakeRetracted(),
                Commands.waitUntil(() -> m_slapdown.getSlapdownAngleRadians() < SuperstructureConstants.kClimbSlapdownMaxAngleRad),
                applyClimbStateAllParallel(ClimbStates.READY)
            );
        }

        /** Extends the hook to climb */
        public Command applyClimbStateHookLocked() {
            return applyClimbStateAllParallel(ClimbStates.LOCKED);
        }

        /** Climb! */
        public Command applyClimbStateClimbed() {
            return applyClimbStateAllParallel(ClimbStates.CLIMBED);
        }

        /* AUTON COMMANDS */
        public Command autonExtendIntake() {
            return applyIntakeStateRollerOnly(IntakeStates.EXTENDED);
        }

        public Command autonRetractIntake() {
            return applyIntakeStateRollerOnly(IntakeStates.RETRACTED);
        }

        public Command teleopInit() {
            return Commands.parallel(
                stopShooting(),
                applyIntakeStateRollerOnly(IntakeStates.EXTENDED)
            );
        }
    }

    @AutoLogOutput(key = "FuelCount/Hub")
    public int getHubFuelCount() {
        return ballCounter.getHubCount();
    }

    @AutoLogOutput(key = "FuelCount/Passing")
    public int getPassingFuelCount() {
        return ballCounter.getPassCount();
    }
}