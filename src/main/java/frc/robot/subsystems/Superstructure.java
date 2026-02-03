package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SuperstructureConstants.FuelManagementStates;
import frc.robot.subsystems.SuperstructureConstants.TurretStates;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.slapdown.SlapdownSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class Superstructure {
    public class MutableSuperState{
        private TurretStates turretState = TurretStates.IDLE;
        private FuelManagementStates fuelManagement = FuelManagementStates.IDLE_RETRACTED;
        
        public TurretStates getTurretState() {
            return turretState;
        }
        
        public void setTurretState(TurretStates newTurretState) {
            turretState = newTurretState;
        }

        public FuelManagementStates getFuelManagement() {
            return fuelManagement;
        }

        public void setTurretState(FuelManagementStates newFuelManagement) {
            fuelManagement = newFuelManagement;
        }

        public MutableSuperState() {
            this.turretState = TurretStates.IDLE;
            this.fuelManagement = FuelManagementStates.IDLE_RETRACTED;
        }

        public MutableSuperState(TurretStates turretState, FuelManagementStates fuelManagement) {
            this.turretState = turretState;
            this.fuelManagement = fuelManagement;
        }
    }
    
    private final TurretSubsystem m_turret;
    private final FlywheelSubsystem m_flyWheel;
    private final HoodSubsystem m_hood;

    private final RollerSubsystem m_roller;
    private final SlapdownSubsystem m_slapdown;
    private final SpindexerSubsystem m_spindexer;
    private final KickerSubsystem m_kicker;

    private final Supplier<Pose2d> m_robotPoseSupplier;

    public Superstructure(
        TurretSubsystem turret,
        FlywheelSubsystem flyWheel,
        HoodSubsystem hood,
        RollerSubsystem roller,
        SlapdownSubsystem slapdown,
        SpindexerSubsystem spindexer,
        KickerSubsystem kicker,
        Supplier<Pose2d> robotPoseSupplier
    ) {
        this.m_turret = turret;
        this.m_flyWheel = flyWheel;
        this.m_hood = hood;
        this.m_roller = roller;
        this.m_slapdown = slapdown;
        this.m_spindexer = spindexer;
        this.m_kicker = kicker;

        this.m_robotPoseSupplier = robotPoseSupplier; 
    }
}