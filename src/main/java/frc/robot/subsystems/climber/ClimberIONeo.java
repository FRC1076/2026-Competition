// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.climber.ClimberConstants.HookConstants;

public class ClimberIONeo implements ClimberIO {
    private final SparkMax m_motor;
    private final SparkMaxConfig m_motorConfig;
    private final SparkClosedLoopController m_climbClosedLoopController;
    private final RelativeEncoder m_climbEncoder;

    private final SparkMax m_hookMotor;
    private final SparkMaxConfig m_hookMotorConfig;
    private final SparkClosedLoopController m_hookClosedLoopController;
    private final RelativeEncoder m_hookEncoder;

    public ClimberIONeo() {
        m_motor = new SparkMax(ClimberConstants.kCANId, MotorType.kBrushless);
        m_motorConfig = new SparkMaxConfig();

        m_hookMotor = new SparkMax(HookConstants.kCANId, MotorType.kBrushless);
        m_hookMotorConfig = new SparkMaxConfig();

        m_motorConfig
            .inverted(ClimberConstants.kMotorInverted)
            .smartCurrentLimit(ClimberConstants.kCurrentLimit)
            .voltageCompensation(ClimberConstants.kVoltageCompensation);
        
        m_motorConfig.encoder
            .positionConversionFactor(ClimberConstants.kPositionConversionFactor)
            .velocityConversionFactor(ClimberConstants.kVelocityConversionFactor);

        m_motorConfig.closedLoop
            .p(ClimberConstants.kP)
            .i(ClimberConstants.kI)
            .d(ClimberConstants.kD);
        
        m_motorConfig.closedLoop.feedForward
            .kS(ClimberConstants.kS)
            .kV(ClimberConstants.kV)
            .kA(ClimberConstants.kA)
            .kG(ClimberConstants.kG);

        m_motorConfig.closedLoop.maxMotion
            .cruiseVelocity(ClimberConstants.kProfileConstraints.maxVelocity)
            .maxAcceleration(ClimberConstants.kProfileConstraints.maxAcceleration);

        m_motorConfig.softLimit
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false); 
        
        m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_climbEncoder = m_motor.getEncoder();
        m_climbEncoder.setPosition(0);

        m_climbClosedLoopController = m_motor.getClosedLoopController();

        m_hookMotorConfig
            .inverted(HookConstants.kMotorInverted)
            .smartCurrentLimit(HookConstants.kCurrentLimit)
            .voltageCompensation(HookConstants.kVoltageCompensation);

        m_hookMotorConfig.encoder
            .positionConversionFactor(HookConstants.kPositionConversionFactor)
            .velocityConversionFactor(HookConstants.kVelocityConversionFactor);

        m_hookMotorConfig.closedLoop
            .p(HookConstants.kP)
            .i(HookConstants.kI)
            .d(HookConstants.kD);

        m_hookMotorConfig.closedLoop.feedForward
            .kS(HookConstants.kS)
            .kV(HookConstants.kV)
            .kA(HookConstants.kA);

        m_hookMotorConfig.closedLoop.maxMotion
            .cruiseVelocity(HookConstants.kCruiseVelocity)
            .maxAcceleration(HookConstants.kMaxAccel);
            
        m_hookMotorConfig.softLimit
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false); 

        m_hookMotor.configure(m_hookMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_hookEncoder = m_hookMotor.getEncoder();
        m_hookEncoder.setPosition(HookConstants.kHookStowedPosition);

        m_hookClosedLoopController = m_hookMotor.getClosedLoopController();
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

    /** Sets the voltage for the hook motor */
    @Override
    public void setHookVoltage(double volts) {
        m_hookMotor.setVoltage(volts);
    }

    /** Sets the setpoint for the hook PID controller */
    @Override
    public void setHookPosition(double positionRadians) {
        m_hookClosedLoopController.setSetpoint(positionRadians, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.appliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.currentAmps = m_motor.getOutputCurrent();
        inputs.positionMeters = m_climbEncoder.getPosition();
        inputs.velocityMPS = m_climbEncoder.getVelocity();

        inputs.hookAppliedVoltage = m_hookMotor.getAppliedOutput() * m_hookMotor.getBusVoltage();
        inputs.hookCurrentAmps = m_hookMotor.getOutputCurrent();
        inputs.hookPositionRadians = m_hookEncoder.getPosition();

        Logger.recordOutput("Climber/PIDTargetRadians", m_climbClosedLoopController.getSetpoint());
    }
}