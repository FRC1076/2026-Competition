// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class TurretSubsystem extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public TurretSubsystem(TurretIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(TurretConstants.kSysIdRampRate).per(Second),
                Volts.of(TurretConstants.kSysIdStepVoltage),
                null,
                (state) -> Logger.recordOutput("Turret/SysIdState", state.toString())
            ), 
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)), 
                (log) ->
                    log.motor("Turret Kraken")
                    .voltage(Volts.of(inputs.motorAppliedVoltage))
                    .angularPosition(Radians.of(inputs.motorPositionRad))
                    .angularVelocity(RadiansPerSecond.of(inputs.motorVelocityRadPerSec)), 
                this,
                null
            )
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }

    public boolean withinTolerance(double targetRadians) {
        return Math.abs(MathUtil.angleModulus(targetRadians - inputs.motorPositionRad)) <= TurretConstants.kPIDToleranceRad;
    }

    public double desaturateTurretPosition(double targetPos) {
        return ((targetPos - TurretConstants.kMinPositionRad)
            % TurretConstants.kAngleRange
            + TurretConstants.kAngleRange)
            % TurretConstants.kAngleRange
            + TurretConstants.kMinPositionRad;
    }

    /** Apply a voltage to the motor with software stops enabled */
    public Command applyVoltage(double volts) {
        return Commands.runOnce(
            () -> io.setVoltage(volts),
            this
        );
    }

    /** Run the motor at a voltage with software stops enabled */
    public Command runVoltage(DoubleSupplier volts) {
        return Commands.run(
            () -> io.setVoltage(volts.getAsDouble()),
            this
        );
    }

    /** Apply a voltage to the motor with software stops disabled */
    public Command applyVoltageUnrestricted(double volts) {
        return Commands.runOnce(
            () -> io.setVoltageNoSoftStops(volts), 
            this
        );
    }

    /** Run the motor at a voltage with software stops disabled */
    public Command runVoltageUnrestricted(DoubleSupplier volts) {
        return Commands.run(
            () -> io.setVoltageNoSoftStops(volts.getAsDouble()), 
            this
        );
    }

    /** Tell the motor to go to a specific position */
    public Command applyPosition(double radians) {
        return Commands.runOnce(
            () -> io.setPosition(desaturateTurretPosition(radians)),
            this
        );
    }

    /** Tell the motor to go to the supplie position ONCE */
    public Command applyPosition(DoubleSupplier radians) {
        return Commands.runOnce(
            () -> io.setPosition(desaturateTurretPosition(radians.getAsDouble())),
            this
        );
    }

    /** Repeatedly tell the motor to go to a specific position */
    public Command runPosition(DoubleSupplier radians) {
        return Commands.run(
            () -> io.setPosition(desaturateTurretPosition(radians.getAsDouble())),
            this
        );
    }

    /** Repeated tell the motor to go to a specific position while the boolean supplier is true */
    public Command runPositionSafe(DoubleSupplier radians, BooleanSupplier isSafe) {
        return Commands.run(
            () -> {
                if (isSafe.getAsBoolean()) {
                    io.setPosition(desaturateTurretPosition(radians.getAsDouble()));
                } else {
                    io.setVoltage(0);
                }
            },
            this
        );
    }

    public Command rezeroTurret() {
        return Commands.runOnce(() -> io.resetPosition());
    }

    public Command sysIdQuasistatic(Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return sysId.dynamic(direction);
    }
}
