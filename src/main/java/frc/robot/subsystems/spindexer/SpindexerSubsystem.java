package frc.robot.subsystems.spindexer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase{
    private SpindexerIO io;
    private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    public SpindexerSubsystem(SpindexerIO io) {
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
        return Commands.runOnce(() -> setVoltage (volts), this);
    }
}
