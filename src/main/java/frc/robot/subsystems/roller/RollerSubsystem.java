package frc.robot.subsystems.roller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem  extends SubsystemBase{
    private RollerIO io;
    private RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

    public RollerSubsystem(RollerIO io) {
        this.io = io;
    }

    public void setVoltage(double volts) {
        this.io.setVoltage(volts);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command applyVoltage(double volts) {
        return Commands.runOnce(() -> setVoltage(volts), this);
    }
}
