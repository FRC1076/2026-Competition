// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.climb.hook;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.climb.climber.ClimberConstants.HookConstants;

public class HookIONeo implements HookIO {
    private final SparkMax m_motor;
    private final SparkMaxConfig m_motorConfig;
    private final SparkClosedLoopController m_climbClosedLoopController;
    private final RelativeEncoder m_climbEncoder;

    public HookIONeo() {
        m_motor = new SparkMax(HookConstants.kCANId, MotorType.kBrushless);
        m_motorConfig = new SparkMaxConfig();

        m_motorConfig
            .inverted(HookConstants.kMotorInverted)
            .smartCurrentLimit(HookConstants.kCurrentLimit)
            .voltageCompensation(HookConstants.kVoltageCompensation);
        
        m_motorConfig.encoder
            .positionConversionFactor(HookConstants.kPositionConversionFactor)
            .velocityConversionFactor(HookConstants.kVelocityConversionFactor);

        m_motorConfig.closedLoop
            .p(HookConstants.kP)
            .i(HookConstants.kI)
            .d(HookConstants.kD);
        
        m_motorConfig.closedLoop.feedForward
            .kS(HookConstants.kS)
            .kV(HookConstants.kV)
            .kA(HookConstants.kA);

        m_motorConfig.closedLoop.maxMotion
            .cruiseVelocity(HookConstants.kCruiseVelocity)
            .maxAcceleration(HookConstants.kMaxAccel);

        m_motorConfig.softLimit
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false); 
        
        m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_climbEncoder = m_motor.getEncoder();
        m_climbEncoder.setPosition(0);

        m_climbClosedLoopController = m_motor.getClosedLoopController();
    }

    /** Sets the voltage for the climber motor */
    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    /** Sets the goal position for climber's PID controller in meters */
    @Override
    public void setPosition(double positionMeters) {
        m_climbClosedLoopController.setSetpoint(positionMeters, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void updateInputs(HookIOInputs inputs) {
        inputs.appliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.currentAmps = m_motor.getOutputCurrent();
        inputs.positionRadians = m_climbEncoder.getPosition();

        Logger.recordOutput("Climber/PIDTargetRadians", m_climbClosedLoopController.getSetpoint());
    }
}