package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class HoodSubsystem extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    private boolean applySoftwareStop = false;

    public HoodSubsystem(HoodIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, Volts.of(1), null,
                (state) -> Logger.recordOutput("Hood/SysIDState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                null, // TODO: do we need logging here?
                this
            )
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        if (applySoftwareStop && inputs.angleRadians >= HoodConstants.kMaxHoodAngleRadians && inputs.appliedVolts > 0) {
            io.setVoltage(0);
        } else if (applySoftwareStop && inputs.angleRadians <= HoodConstants.kMinHoodAngleRadians && inputs.appliedVolts < 0) {
            io.setVoltage(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command applyVoltage(double volts) {
        applySoftwareStop = true;

        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }

    public Command applyVoltageUnrestricted(double volts) {
        applySoftwareStop = false;

        return Commands.runOnce(
            () -> io.setVoltage(volts), 
            this
        );
    }

    public Command applyPosition(double radians) {
        applySoftwareStop = true;

        return Commands.runOnce(
            () -> io.setPosition(MathUtil.clamp(radians, HoodConstants.kMinHoodAngleRadians, HoodConstants.kMaxHoodAngleRadians)), 
            this
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }
}