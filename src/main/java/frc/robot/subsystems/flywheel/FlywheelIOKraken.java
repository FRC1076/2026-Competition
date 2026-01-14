package frc.robot.subsystems.flywheel;

import lib.units.TalonFXUnitConverter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOKraken implements FlywheelIO {
    // Create a motor, configuration, and unit converter here

    // Make a MotionMagic velocity request here

    // Make voltage, velocity, current, and temperature status signals here

    public FlywheelIOKraken() {
        // Instantiate motor here

        // Instantiate configuration and unit converter

        // Configure voltage and current limits

        // Set inverted based on constants

        // Set brake mode based on constants

        // Configure motiom magic based on constants (kP, kI, kD, kS, kV, kA, accleration limit, jerk limit)
        // Make sure to use .fromSIkP(), etc in the unit converter!

        // Apply the configuration to the motor

        // Set up status signals

    }

    @Override
    public void setVoltage(double volts) {
        // Set the voltage of the motor
    }

    @Override
    public void setVelocityRadPerSec(double velocityRadPerSec) {
        // Set the velocity of the motor
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        // Update inputs based on status signals
    }
}