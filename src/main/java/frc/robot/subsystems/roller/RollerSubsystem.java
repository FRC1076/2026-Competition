package frc.robot.subsystems.roller;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem  extends SubsystemBase {
    private RollerIO io;
    private RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

    public RollerSubsystem(RollerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Roller", inputs);
    }

    public Command applyVoltage(double volts) {
        return Commands.runOnce(() -> io.setVoltage(volts), this);
    }

    public Command stop() {
        return applyVoltage(0);
    }
}
