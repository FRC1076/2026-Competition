package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkClosedLoopController;

public class ClimberIOSparkMax implements ClimberIO {
    private final SparkMax m_motor;
    private final SparkMaxConfig m_motorConfig;

    private final RelativeEncoder m_encoder;

    private SparkClosedLoopController m_closedLoopController;

    public ClimberIOSparkMax() {

        m_motor = new SparkMax(ClimberConstants.kCANId, MotorType.kBrushless);
        m_motorConfig = new SparkMaxConfig();

        m_motor.setCANTimeout(250);

        m_motorConfig
            .inverted(ClimberConstants.motorInverted)
            .smartCurrentLimit((int) ClimberConstants.kCurrentLimit)
            .voltageCompensation(ClimberConstants.kVoltageCompensation);

        m_motorConfig.closedLoop
            .pid(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);
        
        m_motorConfig.encoder
            .positionConversionFactor(ClimberConstants.kPositionConversionFactor)
            .velocityConversionFactor(ClimberConstants.kVelocityConversionFactor)
            .quadratureMeasurementPeriod(10)
            .quadratureAverageDepth(2);

        
        m_motor.configure(m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        m_encoder = m_motor.getEncoder();

        m_encoder.setPosition(0);

        m_motor.setCANTimeout(0);

        m_closedLoopController = m_motor.getClosedLoopController();
    }

    @Override
    public void setVoltage(double volts) {
        // we do not need to set PID control to false because we're running it on a closed loop controller, when we manually set the voltage it knows that it's not PID anymore
        m_motor.setVoltage(volts);
    }

    @Override
    public void setPosition(double positionMeters) {
        m_closedLoopController.setSetpoint(positionMeters, SparkMax.ControlType.kPosition);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.appliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.appliedOutput = m_motor.getAppliedOutput();
        inputs.currentAmps = m_motor.getOutputCurrent();

        inputs.climberPosition = m_encoder.getPosition();
        inputs.climberVelocity = m_encoder.getVelocity();
    }

    public void stop() {
        this.setVoltage(0.0);
    }

}