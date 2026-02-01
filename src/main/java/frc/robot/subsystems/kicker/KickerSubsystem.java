package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    public KickerSubsystem(KickerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // Log inputs
        io.updateInputs(inputs);
        Logger.processInputs("Kicker", inputs);
    }

    public Command setVoltage(double volts) {
        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }

    public Command stop() {
        return setVoltage(0);
    }
}
