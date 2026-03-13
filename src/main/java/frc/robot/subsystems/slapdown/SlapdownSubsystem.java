// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.slapdown;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class SlapdownSubsystem extends SubsystemBase{
    private final SlapdownIO io;
    private final SlapdownIOInputsAutoLogged inputs = new SlapdownIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public SlapdownSubsystem(SlapdownIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
            null, Volts.of(1), null,
            (state) -> Logger.recordOutput("Slapdown/SysIdState", state.toString())
            ),

            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                (log) ->
                    log.motor("Slapdown Neo")
                        .voltage(Volts.of(inputs.appliedVoltage))
                        .angularPosition(Radians.of(inputs.angleRadians))
                        .angularVelocity(RadiansPerSecond.of(inputs.velocityRadiansPerSecond)),
                this 
                
            )
        );
    }

    public double getSlapdownAngleRadians() {
        return inputs.angleRadians;
    }

    public boolean withinTolerance(double target) {
        return Math.abs(getSlapdownAngleRadians() - target) < SlapdownConstants.kAngleToleranceRadians;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Slapdown", inputs);
    }

    /** Set the voltage applied to the motor with software stops enabled */
    public Command applyVoltage(double volts) {
        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }

    /** Run the motor at the supplied voltage with software stops enabled */
    public Command runVoltage(DoubleSupplier volts) {
        return Commands.run(
            () -> io.setVoltage(volts.getAsDouble()),
            this
        );
    }

    /** Set the motor to the specified voltage with software stops enabled */
    public Command applyVoltageUnrestricted(double volts) {
        return Commands.runOnce(
            () -> io.setVoltageNoSoftStops(volts),
            this
        );
    }

    /** Run the motor at the supplied voltage with software stops disabled */
    public Command runVoltageUnrestricted(DoubleSupplier volts) {       
        return Commands.run(
            () -> io.setVoltageNoSoftStops(volts.getAsDouble()),
            this
        );
    }

    /** Tell the slapdown to go to the specified position */
    public Command applyPosition(double radians) {
        return Commands.runOnce(
            () -> io.setPosition(MathUtil.clamp(radians, SlapdownConstants.kMinAngleRadians, SlapdownConstants.kMaxAngleRadians)),
            this
        );
    }

    /** Run the slapdown to the supplied position */
    public Command runPosition(DoubleSupplier radians) {
        return Commands.run(
            () -> io.setPosition(MathUtil.clamp(radians.getAsDouble(), SlapdownConstants.kMinAngleRadians, SlapdownConstants.kMaxAngleRadians)),
            this
        );
    }

    /** Tell the slapdown to hold its position but allow it to be somehwat compliant */
    public Command holdPositionWeak(double radians) {
        return Commands.runOnce(
            () -> io.setPositionWeak(MathUtil.clamp(radians, SlapdownConstants.kMinAngleRadians, SlapdownConstants.kMaxAngleRadians)),
            this
        );
    }

    /** Run the slapdown to weakly hold the supplied position */
    public Command runHoldPositionWeak(DoubleSupplier radianSupplier) {
        return Commands.run(
            () -> io.setPositionWeak(MathUtil.clamp(radianSupplier.getAsDouble(), SlapdownConstants.kMinAngleRadians, SlapdownConstants.kMaxAngleRadians)),
            this
        );
    }

    /** Set the current position of the slapdown to zero */
    public Command rezeroSlapdown() {
        return Commands.runOnce(() -> io.rezero());
    }

    public Command sysIdQuasistatic(Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysId.dynamic(direction);
    }
}
