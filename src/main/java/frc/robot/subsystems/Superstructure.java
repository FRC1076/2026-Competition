package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
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

    public void configureStateBasedBindings() {
        new Trigger(() -> m_superState.getTurretState() == TurretStates.AUTOAIM)
            .whileTrue(
                Commands.parallel(
                    Commands.runOnce(() -> updateShootingParams()),
                    m_flywheel.runVelocityPerSec(() -> flywheelTargetSpeedRadPerSec),
                    m_hood.runPosition(() -> m_shootingParams.launchPitchRad()),
                    m_turret.runPosition(() -> m_shootingParams.launchYawRad() - m_robotPoseSupplier.get().getRotation().getRadians())
                )
            );
    }

    private void updateShootingParams() {
        final Pose3d shooterPose = new Pose3d(m_robotPoseSupplier.get()).transformBy(PhysicalConstants.kBotRelativeTurretPose);
        final Pose3d target;
        
        final Pose3d shooterPoseAllianceColorCoordinates = shooterPose.relativeTo(
            Constants.GameConstants.teamColor == Alliance.Blue
                ? new Pose3d()
                : new Pose3d(new Translation3d(PhysicalConstants.FieldConstants.fieldLength, PhysicalConstants.FieldConstants.fieldWidth, 0), new Rotation3d())
        );

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
}