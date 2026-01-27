package frc.robot.subsystems.slapdown;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.revrobotics.spark.SparkClosedLoopController;

public class SlapdownIOSparkMax implements SlapdownIO {
    private final SparkMax m_motor;

    private final SparkMaxConfig m_motorConfig;
    private final RelativeEncoder m_relativeEncoder;

    private ProfiledPIDController m_closedLoopController;

    public SlapdownIOSparkMax() {
        m_motor = new SparkMax(SlapdownConstants.kCANId, MotorType.kBrushless);

        m_motorConfig = new SparkMaxConfig();

        // create motor configurations
        m_motorConfig
            .inverted(SlapdownConstants.kMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) SlapdownConstants.kSmartCurrentLimit);

        m_motorConfig.encoder
            .inverted(true)
            .positionConversionFactor(SlapdownConstants.kPositionConversionFactor)
            .velocityConversionFactor(SlapdownConstants.kVelocityConversionFactor);

        // configure motors
        m_motor.configure(m_motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_relativeEncoder = m_motor.getEncoder();

        //m_closedLoopController = new ProfiledPIDController(SlapdownConstants.kP, SlapdownConstants.kI, SlapdownConstants.kD, SlapdownConstants.kProfileConstraints, SlapdownConstants.kS, SlapdownConstants.kG, SlapdownConstants.kV, SlapdownConstants.kA);
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    @Override
    public SlapdownControlConstants getControlConstants() {
        return realControlConstants;
    }

    @Override
    public void updateInputs(SlapdownIOInputs inputs) {
        inputs.appliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.currentAmps = m_motor.getOutputCurrent();
        /** These calculations are to get around the wraparound of the absolute encoder values
         * If the value is greater than 180 degrees, it will become negative
         * 
         * 1. Subtract the zero offset to get within a range of 270 (-90) degrees to 90 degrees
         * 2. Add 180 degrees
         * 3. Modulo 360 to make the previously negative values, which were technically greater than 180 degrees, less than 180 degrees
         * 4. Subtract 180 degrees to make values negative
         */
        //inputs.angleRadians = ((m_relativeEncoder.getPosition()  - SlapdownConstants.kZeroOffsetRadians + Math.PI) % (2 * Math.PI) - Math.PI);
        inputs.velocityRadiansPerSecond = m_relativeEncoder.getVelocity();
        inputs.angleRadians = m_relativeEncoder.getPosition();
    }

    @Override
    public void stop() {
        this.setVoltage(0.0);
    }

}