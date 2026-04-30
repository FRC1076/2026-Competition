package lib.extendedcommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class VariableWaitCommand extends Command {
    private final Timer m_timer = new Timer();
    private final DoubleSupplier m_durationSupplierSecs;

    public VariableWaitCommand(DoubleSupplier durationSupplierSecs) {
        this.m_durationSupplierSecs = durationSupplierSecs;
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_durationSupplierSecs.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }
}
