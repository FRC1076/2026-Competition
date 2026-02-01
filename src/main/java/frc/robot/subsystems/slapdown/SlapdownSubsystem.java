package frc.robot.subsystems.slapdown;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class SlapdownSubsystem extends SubsystemBase{
    private final SlapdownIO io;
    private final SlapdownIOInputsAutoLogged inputs = new SlapdownIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    private boolean softwareStopsEnabled = true;

    public SlapdownSubsystem(SlapdownIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
            null, Volts.of(1), null,
            (state) -> Logger.recordOutput("Slapdown/SysIdState", state.toString())
            ),

            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                (log) ->
                    log.motor("Slapdown Neo")
                        .voltage(Volts.of(inputs.appliedVoltage))
                        .angularPosition(Radians.of(inputs.angleRadians))
                        .angularVelocity(RadiansPerSecond.of(inputs.velocityRadiansPerSecond)),
                this 
                
            )
        );
    }

    @Override
    public void periodic() {
        if (softwareStopsEnabled) {
            if (inputs.angleRadians >= SlapdownConstants.kMaxAngleRadians && inputs.appliedVoltage > 0) {
                io.setVoltage(0);
            } else if (inputs.angleRadians <= SlapdownConstants.kMinAngleRadians && inputs.appliedVoltage < 0) {
                io.setVoltage(0);
            }
        }

        io.updateInputs(inputs);
        Logger.processInputs("Slapdown", inputs);

        Logger.recordOutput("Slapdown/SoftwareStopsEnabled", softwareStopsEnabled);
    }

    public Command applyVoltage(double volts) {
        softwareStopsEnabled = true;

        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }

    public Command applyVoltageUnrestricted(double volts) {
        softwareStopsEnabled = false;
        
        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }

    public Command applyPosition(double radians) {
        softwareStopsEnabled = true;

        return Commands.runOnce(
            () -> io.setPosition(MathUtil.clamp(radians, SlapdownConstants.kMinAngleRadians, SlapdownConstants.kMaxAngleRadians)),
            this
        );
    }

    public Command sysIdQuasistatic(Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysId.dynamic(direction);
    }
}
