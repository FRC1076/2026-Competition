// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.kicker;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    public KickerSubsystem(KickerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // Log inputs
        io.updateInputs(inputs);
        Logger.processInputs("Kicker", inputs);
    }
    /** set the motors to specific voltage */
    public Command applyVoltage(double volts) {
        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }

    /** Run the Kicker motor at supplied voltage */
    public Command runVoltage(DoubleSupplier volts) {
        return Commands.run(
            () -> io.setVoltage(volts.getAsDouble()),
            this
        );
    }

    /** Stop the Kickre's motor */
    public Command stop() {
        return applyVoltage(0);
    }

    /** returns current voltage */
    public double getVoltage() {
        return inputs.appliedVoltage;
    }
}
