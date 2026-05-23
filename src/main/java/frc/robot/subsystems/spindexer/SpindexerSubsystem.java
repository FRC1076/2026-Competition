// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.spindexer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.control.SimpleMotorAsymmetricPFFController;

public class SpindexerSubsystem extends SubsystemBase{
    private SpindexerIO io;
    private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    private boolean runAsymPFF = false;
    private final SimpleMotorAsymmetricPFFController m_asymPFF;

    public SpindexerSubsystem(SpindexerIO io) {
        this.io = io;

        m_asymPFF = new SimpleMotorAsymmetricPFFController(
            SpindexerConstants.kP,
            SpindexerConstants.kS,
            SpindexerConstants.kV,
            SpindexerConstants.kA,
            SpindexerConstants.kPFFDeadband);
    }
    
    @Override 
    public void periodic() {
        io.updateInputs(inputs);

        if (runAsymPFF) {
            io.setTorque(m_asymPFF.calculateVelocity(inputs.velocityRadPerSec));
        }

        Logger.processInputs("Spindexer", inputs);
    }

    private void setVoltage(double volts) {
        runAsymPFF = false;
        io.setVoltage(volts);
    }

    private void setVelocity(double velocity) {
        runAsymPFF = false;
        io.setVelocity(velocity);
    }

    private void setVelocityAsym(double velocity) {
        runAsymPFF = true;
        m_asymPFF.setSetpoint(velocity);
    }

    /**Set the motor to specific voltage */
    public Command applyVoltage(double volts) {
        return Commands.runOnce(
            () -> setVoltage(volts),
            this
        );
    }

    /** Run motors at the specific voltage */
    public Command runVoltage(DoubleSupplier volts) {
        return Commands.run(
            () -> setVoltage(volts.getAsDouble()),
            this
        );
    }

    /** Set the spindexer to the specified velocity */
    public Command applyVelocity(double radPerSec) {
        return Commands.runOnce(
            () -> setVelocityAsym(radPerSec),
            this
        );
    }

    /** Set the spindexer to the specified velocity */
    public Command applyVelocity(DoubleSupplier radPerSec) {
        return Commands.runOnce(
            () -> setVelocityAsym(radPerSec.getAsDouble()),
            this
        );
    }

    /** Run the spindexer at the supplied velocity */
    public Command runVelocity(DoubleSupplier radPerSec) {
        return Commands.run(
            () -> setVelocityAsym(radPerSec.getAsDouble()),
            this
        );
    }

    /** Gets motor's current voltage */
    public double getVoltage() {
        return inputs.appliedVoltage;
    }

    /** Gets motor's current velocity in rad/s */
    public double getVelocityRadPerSec() {
        return inputs.velocityRadPerSec;
    }
}
