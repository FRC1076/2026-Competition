package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


public class FlywheelSubsystem extends SubsystemBase {
    // Declare a FlywheelIO, FlywheelIOInputsAutoLogged, and SysIdRoutine
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public FlywheelSubsystem(FlywheelIO io) {
        // Instatiate FlywheelIO
        this.io = io;

        // Set up SysId
        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, 
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)), 
                (log) ->
                    log.motor("Flywheel Kraken")
                    .voltage(Volts.of(inputs.appliedVoltage))
                    .angularVelocity(RadiansPerSecond.of(inputs.velocityRadiansPerSecond)), 
                this
            )
        );

    }

    public boolean atSetpoint(double targetRadiansPerSecond) {
        return Math.abs(inputs.velocityRadiansPerSecond - targetRadiansPerSecond) < FlywheelConstants.kSetpointToleranceRadPerSec;
    }

    @Override
    public void periodic() {
        // Update inputs and log them
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
    }

    /** Set the voltage of the motor */
    public Command applyVoltage(double volts) {
        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }

    /** Run the motor at the supplied voltage */
    public Command runVoltage(DoubleSupplier volts) {
        return Commands.run(
            () -> io.setVoltage(volts.getAsDouble()),
            this
        );
    }

    /** Tell the flywheel to run at the specified velocity */
    public Command applyVelocityPerSec(double radPerSec) {
        return Commands.runOnce(
            () -> io.setVelocityRadPerSec(radPerSec),
            this
        );
    }

    /** Run the wheel at the supplied velocity */
    public Command applyVelocityPerSec(DoubleSupplier radPerSec) {
        return Commands.runOnce(
            () -> io.setVelocityRadPerSec(radPerSec.getAsDouble()),
            this
        );
    }

    // Make SysId quasistatic and dynamic commands
    public Command sysIdQuasistatic(Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysId.dynamic(direction);
    }
}
