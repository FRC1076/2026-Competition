package frc.robot.subsystems.slapdown;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SlapdownSubsystem extends SubsystemBase{
    private final SlapdownIO io;
    private final SlapdownIOInputsAutoLogged inputs = new SlapdownIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public SlapdownSubsystem(SlapdownIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
            null, Volts.of(1), null,
            (state) -> Logger.recordOutput("Slapdown/SysIdState", state.toString())
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
        Logger.processInputs("Slapdown", inputs);
    }

    public Command applyVoltage(double volts){
        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }
}
