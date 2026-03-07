// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.spindexer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase{
    private SpindexerIO io;
    private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    public SpindexerSubsystem(SpindexerIO io) {
        this.io = io;
    }
    
    @Override 
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Spindexer", inputs);
    }

    /**Set the motor to specific voltage */
    public Command applyVoltage(double volts) {
        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }

    /** Run motors at the specific voltage */
    public Command runVoltage(DoubleSupplier volts) {
        return Commands.run(
            () -> io.setVoltage(volts.getAsDouble()),
            this
        );
    }

    /** Set the spindexer to the specified velocity */
    public Command applyVelocity(double radPerSec) {
        return Commands.runOnce(
            () -> io.setVelocity(radPerSec),
            this
        );
    }

    /** Run the spindexer at the supplied velocity */
    public Command runVelocity(DoubleSupplier radPerSec) {
        return Commands.run(
            () -> io.setVelocity(radPerSec.getAsDouble()),
            this
        );
    }

    /** Gets motor's current voltage */
    public double getVoltage() {
        return inputs.appliedVoltage;
    }
}
