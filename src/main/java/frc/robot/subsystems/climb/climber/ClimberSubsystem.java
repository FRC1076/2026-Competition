// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.climb.climber;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.climber.ClimberIOInputsAutoLogged;

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
                (log) -> 
                    log.motor("Climber Neo")
                    .voltage(Volts.of(inputs.appliedVoltage))
                    .linearPosition(Meters.of(inputs.positionMeters))
                    .linearVelocity(MetersPerSecond.of(inputs.velocityMPS)), 
                this
            )
        );

    }

    @Override
    public void periodic() {
        io.periodic();
        // Update inputs and log them
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    /** Set the motor to the specified voltage */
    public Command applyVoltage(double volts) {
        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }

    /** Run the climber motor at the supplied voltage */
    public Command runVoltage(DoubleSupplier volts) {
        return Commands.run(
            () -> io.setVoltage(volts.getAsDouble()),
            this
        );
    }

    /** Tell the climber to go to the specified position */
    public Command applyPosition(double meters) {
        return Commands.runOnce(
            () -> io.setPosition(meters),
            this
        );
    }

    /** Run the climber at the supplied position */
    public Command runPosition(DoubleSupplier meters) {
        return Commands.run(
            () -> io.setPosition(meters.getAsDouble()),
            this
        );
    }

    /** Set the hook motor to run at a voltage */
    public Command applyHookVoltage(double volts) {
        return Commands.runOnce(
            () -> io.setHookVoltage(volts)
        );
    }

    /** Tell the hook motor to run at the supplied voltage */
    public Command runHookVoltage(DoubleSupplier volts) {
        return Commands.run(
            () -> io.setHookVoltage(volts.getAsDouble())
        );
    }

    /** Tell to hook to go to the position in radians */
    public Command applyHookPosition(double radians) {
        return Commands.runOnce(
            () -> io.setHookPosition(radians)
        );
    }

    /** Tell the hook to run at the supplied position in radians */
    public Command applyHookPosition(DoubleSupplier radians) {
        return Commands.run(
            () -> io.setHookPosition(radians.getAsDouble())
        );
    }

    public Command sysIdQuasistatic(Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysId.dynamic(direction);
    }

    /** Stops the climber's motors */
    public Command stop() {
        return Commands.sequence(
            Commands.runOnce(() -> io.setVoltage(0), this),
            Commands.runOnce(() -> io.setHookVoltage(0))
        );
    }   
}

    
