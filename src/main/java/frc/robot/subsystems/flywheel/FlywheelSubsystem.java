package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


public class FlywheelSubsystem extends SubsystemBase {
    // Declare a FlywheelIO, FlywheelIOInputsAutoLogged, and SysIdRoutine
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    private final SysIdRoutine sysid;
    public FlywheelSubsystem(FlywheelIO io) {
        // Instatiate FlywheelIO
        this.io = io;

        // Set up SysId
        sysid = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, 
                (state) -> Logger.recordOutput("Flywheel/SysIDState", state.toString())
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
        // Update inputs and log them
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
    }

    // Make command to set voltage
    public Command applyVoltage(double volts)
    {
        return Commands.runOnce(() -> io.setVoltage(volts), this);
    }
    // Make command to set velocity in radians per second
    public Command applyVelocityPerSec(double velocity)
    {
        return Commands.runOnce(() -> io.setVelocityRadPerSec(velocity), this);
    }
    // Make SysId quasistatic and dynamic commands
    public Command flywheelSysIDQuasistatic(Direction direction)
    {
        return sysid.quasistatic(direction);
    }

    public Command flywheelDynamics(Direction direction)
    {
        return sysid.dynamic(direction);
    }
}
