package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class HoodSubsystem extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public HoodSubsystem(HoodIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, Volts.of(1), null,
                (state) -> Logger.recordOutput("Hood/SysIDState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                null,
                this
            )
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command applyVoltage(double volts) {
        if (inputs.angleRadians >= HoodConstants.kMaxHoodAngleRadians && volts > 0) {
            volts = 0;
        } else if (inputs.angleRadians <= HoodConstants.kMinHoodAngleRadians && volts < 0) {
            volts = 0;
        }

        final double voltage = volts;

        return Commands.runOnce(
            () -> io.setVoltage(voltage),
            this
        );
    }

    public Command applyVoltageUnrestricted(double volts) {
        return Commands.runOnce(
            () -> io.setVoltage(volts), 
            this
        );
    }

    public Command applyPosition(double radians) {
        if (inputs.angleRadians > HoodConstants.kMaxHoodAngleRadians) {
            radians = HoodConstants.kMaxHoodAngleRadians;
        } else if (inputs.angleRadians < HoodConstants.kMinHoodAngleRadians) {
            radians = HoodConstants.kMinHoodAngleRadians;
        }

        final double targetRadians = radians;

        return Commands.runOnce(
            () -> io.setPosition(targetRadians), 
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