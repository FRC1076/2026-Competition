package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.PhysicalConstants;
import frc.robot.subsystems.SuperstructureConstants.FuelManagementStates;
import frc.robot.subsystems.SuperstructureConstants.TurretStates;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.slapdown.SlapdownSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import lib.ballistic.CommonShotSolution;
import lib.ballistic.HoundSOTMCalculator;

public class Superstructure {
    public class MutableSuperState{
        private TurretStates turretState = TurretStates.IDLE;
        private FuelManagementStates fuelManagement = FuelManagementStates.IDLE_RETRACTED;
        
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

        /** Returns the state of the part of robot that manages fuel. */
        public FuelManagementStates getFuelManagement() {
            return fuelManagement;
        }

        /** Sets the target state of part of the robot that manages fuel
         *  (note: this does not actually affect the physical robot, it only
         *   affects the variable in the code)
          */
        public void setFuelManagementState(FuelManagementStates newFuelManagement) {
            fuelManagement = newFuelManagement;
        }

        /** Default constructor. Sets all variables to IDLE */
        public MutableSuperState() {
            this.turretState = TurretStates.IDLE;
            this.fuelManagement = FuelManagementStates.IDLE_RETRACTED;
        }
        
        /** Instantiates a MutableSuperState with the specified TurretState and FuelManagementeState */
        public MutableSuperState(TurretStates turretState, FuelManagementStates fuelManagement) {
            this.turretState = turretState;
            this.fuelManagement = fuelManagement;
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

        m_shootingParams = new CommonShotSolution(0, 0, 0);
    }

    /** Configure trigger bindings that are based upon the state of the robot. */
    public void configureStateBasedBindings() {
        new Trigger(() -> m_superState.getTurretState().kIsAutoAim)
            .whileTrue(
                Commands.repeatingSequence(
                    Commands.runOnce(() -> updateShootingParams()),
                    Commands.parallel(
                        m_flywheel.applyVelocity(flywheelTargetSpeedRadPerSec),
                        m_hood.applyPosition(m_shootingParams.launchPitchRad()),
                        m_turret.applyPosition(m_shootingParams.launchYawRad() - m_robotPoseSupplier.get().getRotation().getRadians())
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
        
        final Pose3d shooterPoseAllianceColorCoordinates = shooterPose.relativeTo(SuperstructureConstants.kAllianceOrigin);

        if (shooterPoseAllianceColorCoordinates.getX() <= PhysicalConstants.FieldConstants.LinesVertical.allianceZone) {
            target = SuperstructureConstants.kHubTarget;
        } else if (shooterPose.getY() <= PhysicalConstants.FieldConstants.LinesHorizontal.center) {
            target = SuperstructureConstants.kRightPassingTarget;
        } else {
            target = SuperstructureConstants.kLeftPassingTarget;
        }
        
        flywheelTargetSpeedRadPerSec = SuperstructureConstants.kDistanceToFlywheelSpeedMap.get(shooterPose.getTranslation().getDistance(target.getTranslation()));
        m_shootingParams = HoundSOTMCalculator.solveShootOnTheFly(
            shooterPose, 
            target,
            m_robotVelocitySupplier.get(),
            m_flywheel.getLinearVelocityMPS(),
            SuperstructureConstants.kAutoAimMaxIterations,
            SuperstructureConstants.kAutoAimTimeToleranceSeconds
        );
    }

    /** Apply the passed in state to the slapdown, intake rollers, spindexer, and kicker,
     *  all at the same time. */
    public Command applyFuelManagementStateAllParallel(FuelManagementStates state) {
        m_superState.setFuelManagementState(state);

        return Commands.parallel(
            m_slapdown.applyPosition(state.slapdownRadians),
            m_roller.applyVoltage(state.rollerVoltage),
            m_spindexer.applyVoltage(state.spindexerVoltage),
            m_kicker.applyVoltage(state.kickerVoltage)
        );
    }

    /** Apply the passed in state to the turret, hood, and flywheel, all at the same time. */
    public Command applyTurretStateAllParallel(TurretStates state){
        m_superState.setTurretState(state);

        return Commands.parallel(
            m_turret.applyPosition(state.kTurretAngleRadians),
            m_flywheel.applyVelocity(state.kFlywheelRadPerSec),
            m_hood.applyPosition(state.kHoodAngleRadians)
        );
    }

    /** Sets the turret state to the desired item such so that a state-based trigger can take control */
    public Command setTurretState(TurretStates state) {
        return Commands.runOnce(() -> m_superState.setTurretState(state));
    }

    /** Public Commands to be used to bind to triggers in RobotContainer */
    public class SuperstructureCommandFactory{
        private final Superstructure m_superstructure;

        public SuperstructureCommandFactory(Superstructure superstructure){
            this.m_superstructure = superstructure;
        }

        /** Apply the IDLE_RETRACTED fuel management state */
        public Command applyFuelManagementIdleRetracted(){
            return m_superstructure.applyFuelManagementStateAllParallel(FuelManagementStates.IDLE_RETRACTED);
        }

        /** Apply the IDLE_EXTENDED fuel management state */
        public Command applyFuelManagementIdleExtended(){
            return m_superstructure.applyFuelManagementStateAllParallel(FuelManagementStates.IDLE_EXTENDED);
        }

        /** Apply the INTAKE fuel management state */
        public Command applyFuelManagementIntake(){
            return m_superstructure.applyFuelManagementStateAllParallel(FuelManagementStates.INTAKE_FUEL);
        }

        /** Apply the INTAKE_INDEX fuel management state. Can be used for shooting. */
        public Command applyFuelManagementIntakeIndex(){
            return m_superstructure.applyFuelManagementStateAllParallel(FuelManagementStates.INTAKE_INDEX_FUEL);
        }

        /** Apply the INDEX_RETRACTED fuel management state. Can be used for shooting */
        public Command applyIndexRetracted(){
            return m_superstructure.applyFuelManagementStateAllParallel(FuelManagementStates.INDEX_FUEL_RETRACTED);
        }

        /** Apply the INDEX_EXTENDED fuel management state. Can be used for shooting. */
        public Command applyIndexExtended(){
            return m_superstructure.applyFuelManagementStateAllParallel(FuelManagementStates.INDEX_FUEL_EXTENDED);
        }

        /** Apply the IDLE turret state */
        public Command applyTurretStatesIdle(){
            return m_superstructure.applyTurretStateAllParallel(TurretStates.IDLE);
        }

        /** Apply the MANUAL turret state */
        public Command applyTurretStatesManual(){
            return m_superstructure.applyTurretStateAllParallel(TurretStates.MANUAL);
        }

       /** Apply the ATOUAIM turret state */
       public Command applyTurretStatesAutoAim(){
            return m_superstructure.setTurretState(TurretStates.AUTOAIM);
       }
       /** Apply the HUB_PREALIGNED_LOCATION turret state */
       public Command applyTurretStatesHubPreAlignedLocation(){
            return m_superstructure.applyTurretStateAllParallel(TurretStates.HUB_PREALIGNED_LOCATION);
       }
       /** Apply the POINT_DIRECTLY_BACK_FOR_PASSING turret state */
       public Command applyTurretStatesPointDirectlyBackForPassing(){
            return m_superstructure.applyTurretStateAllParallel(TurretStates.POINT_DIRECTLY_BACK_FOR_PASSING);
       }
        
    }
}