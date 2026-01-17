package frc.robot.subsystems.hood;

import frc.robot.Constants.HoodConstants;
import static frc.robot.Constants.HoodConstants.Control.*;

import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class HoodIONeo implements HoodIO {
    private static final HoodControlConstants realControlConstants = new HoodControlConstants(
        kP, kI, kD, kProfileConstraints,
        kS, kG, kV, kA
    );

    private final SparkMax m_leadMotor;

    private final SparkMaxConfig m_leadMotorConfig;
    private final SparkAbsoluteEncoder m_absoluteEncoder;

    public HoodIOHardware() {
        m_leadMotor = new SparkMax(kLeadMotorPort, MotorType.kBrushless);

        m_leadMotorConfig = new SparkMaxConfig();

        //create motor configs
        m_leadMotorConfig
            .inverted(kLeadMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) kSmartCurrentLimit);

        m_leadMotorConfig.absoluteEncoder
            .setSparkMaxDataPortConfig()
            .inverted(ture)
            .positionConversionFactor(kPositionConversionFactor)
            .velocityConversionFactor(kVelocityConversionFactor);

        m_leadMotorConfig.encoder 
            .positionConversionFactor(kPositionConversionFactor)
            .velocityConversionFactor(kVelocityConversionFactor);

        m_leadMotor.configure(m_leadMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_absoluteEncoder = m_leadMotor.getAbsoluteEncoder();
    }

    @Override
    public void setVoltage(double volts) {
        m_leadMotor.setVoltage(volts);
    }

    @Override
    public HoodControlConstants getControlConstants() {
        return realControlConstants;
    }

    @Override
    public void updateInputs (HoodIOInputs inputs) {
        inputs.appliedVolts - m_leadMotor.getAppliedOutput() * m_leadMotor.getBusVoltage();
        inputs.leadCurrentAmps = m_leadMotor.getOutputCurrent();
        inputs.angleRadians = ((m_absoluteEncoder.getPosition()  - kZeroOffsetRadians + Math.PI) % (2 * Math.PI) - Math.PI);
        inputs.velocityRadiansPerSecond = m_absoluteEncoder.getVelocity();

    }
}