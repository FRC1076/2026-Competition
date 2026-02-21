// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class HoodSubsystem extends SubsystemBase {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    private boolean applySoftwareStop = false;

    public HoodSubsystem(HoodIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, Volts.of(1), null,
                (state) -> Logger.recordOutput("Hood/SysIDState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                (log) ->
                    log.motor("Hood Neo 550")
                    .voltage(Volts.of(inputs.appliedVolts))
                    .angularPosition(Radians.of(inputs.angleRadians))
                    .angularVelocity(RadiansPerSecond.of(inputs.velocityRadiansPerSecond)),
                this
            )
        );
    }

    public void setSoftwareStop(boolean enabled) {
        applySoftwareStop = enabled;
    }

    public Command applySoftwareStop(boolean enabled) {
        return Commands.runOnce(() -> setSoftwareStop(enabled));
    }

    @Override
    public void periodic() {
        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        if (applySoftwareStop && inputs.angleRadians >= HoodConstants.kMaxHoodAngleRadians && inputs.appliedVolts > 0) {
            io.setVoltage(0);
        } else if (applySoftwareStop && inputs.angleRadians <= HoodConstants.kMinHoodAngleRadians && inputs.appliedVolts < 0) {
            io.setVoltage(0);
        }
    }

    /** Set the motor to a voltage with software stops enabled */
    public Command applyVoltage(double volts) {
        return Commands.sequence(
            applySoftwareStop(true),
            Commands.runOnce(() -> io.setVoltage(volts), this)
        );
    }

    /** Run the motor at the supplied voltage with software stops enabled */
    public Command runVoltage(DoubleSupplier volts) {
        return Commands.sequence(
            applySoftwareStop(true),
            Commands.run(() -> io.setVoltage(volts.getAsDouble()), this)
        );
    }

    /** Set the motor to a voltage with software stops disabled */
    public Command applyVoltageUnrestricted(double volts) {
        return Commands.sequence(
            applySoftwareStop(false),
            Commands.runOnce(() -> io.setVoltage(volts), this)
        );
    }

    /** Run the motor at the supplied voltage with software stops disabled */
    public Command runVoltageUnrestricted(DoubleSupplier volts) {
        return Commands.sequence(
            applySoftwareStop(false),
            Commands.run(() -> io.setVoltage(volts.getAsDouble()), this)
        );
    }

    /** Apply a position to the hood */
    public Command applyPosition(double radians) {
        return Commands.sequence(
            applySoftwareStop(true),
            Commands.runOnce(() -> io.setPosition(radians), this)
        );
    }

    /** Apply the supplied position to the hood ONCE and then ends */
    public Command applyPosition(DoubleSupplier radians) {
        return Commands.sequence(
            applySoftwareStop(true),
            Commands.runOnce(() -> io.setPosition(radians.getAsDouble()), this)
        );
    }

    /** Repeatedly tell the hood to go to a position based on the supplier */
    public Command runPosition(DoubleSupplier radians) {
        return Commands.sequence(
            applySoftwareStop(true),
            Commands.run(() -> io.setPosition(radians.getAsDouble()), this)
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }
}