package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import frc.robot.PhysicalConstants;
import frc.robot.subsystems.SuperstructureConstants.IndexStates;
import frc.robot.subsystems.SuperstructureConstants.IntakeStates;
import frc.robot.subsystems.SuperstructureConstants.TurretStates;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.slapdown.SlapdownSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

import lib.ballistic.CommonShotSolution;
// import lib.ballistic.HoundSOTMCalculator;
import lib.ballistic.MechAdvSOTMCalculator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Superstructure {
    public class MutableSuperState{
        private TurretStates turretState = TurretStates.IDLE;
        private IntakeStates intake = IntakeStates.RETRACTED;
        private IndexStates index = IndexStates.IDLE;
        
        /** Returns the state of the turret assembly */
        public TurretStates getTurretState() {
            return turretState;
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

        /** Sets the Intake state
         *  (note: this does not actually affect the physical robot, it only
         *   affects the variable in the code)
          */
        public void setIntakeState(IntakeStates newIntakeState) {
            intake = newIntakeState;
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

    private final Supplier<Pose2d> m_robotPoseSupplier;
    private final Supplier<ChassisSpeeds> m_robotVelocitySupplier;

    private final MutableSuperState m_superState;
    private final SuperstructureCommandFactory m_commandFactory;

    private CommonShotSolution m_shootingParams;
    private double flywheelTargetSpeedRadPerSec;

    public Superstructure(
        TurretSubsystem turret,
        FlywheelSubsystem flywheel,
        HoodSubsystem hood,
        RollerSubsystem roller,
        SlapdownSubsystem slapdown,
        SpindexerSubsystem spindexer,
        KickerSubsystem kicker,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisSpeeds> robotVelocitySupplier
    ) {
        this.m_turret = turret;
        this.m_flywheel = flywheel;
        this.m_hood = hood;
        this.m_roller = roller;
        this.m_slapdown = slapdown;
        this.m_spindexer = spindexer;
        this.m_kicker = kicker;

        this.m_robotPoseSupplier = robotPoseSupplier; 
        this.m_robotVelocitySupplier = robotVelocitySupplier;

        this.m_superState = new MutableSuperState();

        this.m_commandFactory = new SuperstructureCommandFactory(this);

        m_shootingParams = new CommonShotSolution(0, 0, 0);
    }

    /** Get the Command factory */
    public SuperstructureCommandFactory getCommandFactory() {
        return m_commandFactory;
    }

    /** Configure trigger bindings that are based upon the state of the robot. */
    public void configureStateBasedBindings() {
        new Trigger(() -> m_superState.getTurretState().kIsAutoAim)
            .whileTrue(
                Commands.repeatingSequence(
                    Commands.runOnce(() -> updateShootingParams()),
                    Commands.parallel(
                        m_flywheel.applyVelocity(m_shootingParams.launchSpeedRadPerSec()),
                        m_hood.applyPosition(m_shootingParams.launchPitchRad()),
                        applyTurretPositionSafe(MathUtil.angleModulus(m_shootingParams.launchYawRad() - m_robotPoseSupplier.get().getRotation().getRadians()))
                    )
                )
            );
    }

    /** Update parameters saved to m_shootingParams and flywheelTargetRadiansPerSecond
     *  based upon the current pose of the robot.
     */
    private void updateShootingParams() {
        final Pose3d shooterPose = new Pose3d(m_robotPoseSupplier.get()).transformBy(PhysicalConstants.kBotRelativeTurretPose);
        final Pose3d target;
        final boolean targetIsHub;
        
        final Pose3d shooterPoseAllianceColorCoordinates = shooterPose.relativeTo(SuperstructureConstants.kAllianceOrigin);

        if (shooterPoseAllianceColorCoordinates.getX() <= PhysicalConstants.FieldConstants.LinesVertical.allianceZone) {
            target = SuperstructureConstants.kHubTarget;
            targetIsHub = true;
        } else if (shooterPose.getY() <= PhysicalConstants.FieldConstants.LinesHorizontal.center) {
            target = SuperstructureConstants.kRightPassingTarget;
            targetIsHub = false;
        } else {
            target = SuperstructureConstants.kLeftPassingTarget;
            targetIsHub = false;
        }

        
        /* TechHOUNDs option. Commented out to test Mechanical Advantage's option. * /
        m_shootingParams = HoundSOTMCalculator.solveShootOnTheFly(
            shooterPose, 
            target,
            m_robotVelocitySupplier.get(),
            m_flywheel.getLinearVelocityMPS(),
            SuperstructureConstants.kAutoAimMaxIterations,
            SuperstructureConstants.kAutoAimTimeToleranceSeconds
        ); */

        /* Mechanical Advantage option */
        m_shootingParams = MechAdvSOTMCalculator.calculate(
            shooterPose.toPose2d(),
            target.toPose2d(),
            m_robotVelocitySupplier.get(),
            m_robotPoseSupplier.get().getRotation(),
            targetIsHub
        );
    }

    public MutableSuperState getSuperState() {
        return m_superState;
    }

    /** Returns whether or not the flywheel is within the tolerance of the target shooting state */
    public BooleanSupplier isReadyToShoot() {
        return () -> m_flywheel.readyToShoot(flywheelTargetSpeedRadPerSec);
    }

    /** Returns whether or not the slapdown is too high to move the turret */
    public BooleanSupplier safeToMoveTurret() {
        return () -> m_slapdown.getSlapdownAngleRadians() < SuperstructureConstants.kTurretMoveSlapdownAngleLimitRad;
    }

    /** Applies the target state to the turret if it is safe to do so. If unsafe, does nothing. */
    public Command applyTurretPositionSafe(double radians) {
        return Commands.either(
            m_turret.applyPosition(radians),
            Commands.none(),
            safeToMoveTurret()
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

    public Command applyIntakeStateAllParallel(IntakeStates state) {
        return Commands.parallel(
            setIntakeState(state),
            m_slapdown.applyPosition(state.kSlapdownAngle),
            m_roller.applyVoltage(state.kRollerVoltage)
        );
    }

    public Command applyIndexStateAllParallel(IndexStates state) {
        return Commands.parallel(
            setIndexState(state),
            m_spindexer.applyVoltage(state.kSpindexerVoltage),
            m_kicker.applyVoltage(state.kKickerVoltage)
        );
    }

    /** Apply the passed in state to the turret, hood, and flywheel, all at the same time. */
    public Command applyTurretStateAllParallel(TurretStates state) {
        return Commands.parallel(
            setTurretState(state),
            applyTurretPositionSafe(state.kTurretAngleRadians),
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

        /** Index fuel for shooting */
        public Command startIndexing() {
            return m_superstructure.applyIndexStateAllParallel(IndexStates.INDEXING);
        }

        /** Set the spindexer and kicker to idle */
        public Command stopIndexing() {
            return m_superstructure.applyIndexStateAllParallel(IndexStates.IDLE);
        }

        public Command reverseEverything() {
            return Commands.parallel(
                m_superstructure.applyTurretStateFlywheelOnly(TurretStates.REVERSE),
                m_superstructure.applyIndexStateAllParallel(IndexStates.REVERSE),
                m_superstructure.applyIntakeStateAllParallel(IntakeStates.REVERSE)
            );
        }

        /** Shoot fuel. Either goes to AUTOAIM_SHOOT for turret,
         *  or does nothing for turret depending on if AUTOAIM is active.  */
        public Command shoot() {
            return Commands.either(
                Commands.sequence(
                    shootWithAutoaim(),
                    Commands.waitUntil(isReadyToShoot()),
                    startIndexing()
                ),
                Commands.sequence(
                    m_flywheel.applyVoltage(9),
                    startIndexing()
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
                    stopIndexing(),
                    initiateAutoaim()
                ), 
                stopIndexing(), 
                () -> m_superstructure.getSuperState().getTurretState().kIsAutoAim
            );
        }
    }
}