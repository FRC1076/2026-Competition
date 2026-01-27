package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, 
                (state) -> Logger.recordOutput("Climber/SysIdState", state.toString())
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
        Logger.processInputs("Climber", inputs);
    }

    public Command applyVoltage(double volts) {
        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }

    public Command applyPosition(double meters) {
        return Commands.runOnce(
            () -> io.setPosition(meters),
            this
        );
    }

    public Command sysIdQuasistatic(Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysId.dynamic(direction);
    }

    public Command applyStop() {
        return Commands.runOnce(
            () -> io.stop(),
            this
        );
    }   
}

    
