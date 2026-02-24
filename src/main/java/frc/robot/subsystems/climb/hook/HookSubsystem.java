// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.climb.hook;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class HookSubsystem extends SubsystemBase {
    private final HookIO io;
    private final HookIOInputsAutoLogged inputs = new HookIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public HookSubsystem(HookIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, 
                (state) -> Logger.recordOutput("ClimbHook/SysIdState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)), 
                (log) -> 
                    log.motor("Climber Hook")
                    .voltage(Volts.of(inputs.appliedVoltage))
                    .angularPosition(Radians.of(inputs.positionRadians))
                    .angularVelocity(RadiansPerSecond.of(inputs.velocityRadPerSec)), 
                this
            )
        );

    }

    @Override
    public void periodic() {
        io.periodic();
        // Update inputs and log them
        io.updateInputs(inputs);
        Logger.processInputs("ClimbHook", inputs);
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
    public Command applyPosition(double radians) {
        return Commands.runOnce(
            () -> io.setPosition(radians),
            this
        );
    }

    /** Run the climber at the supplied position */
    public Command runPosition(DoubleSupplier radians) {
        return Commands.run(
            () -> io.setPosition(radians.getAsDouble()),
            this
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
        return Commands.runOnce(() -> io.setVoltage(0), this);
    }   
}

    
