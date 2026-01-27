package frc.robot.subsystems.kicker;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class KickerIOHardware implements KickerIO{
    private SparkMax m_motor; 

public KickerIOHardware() {
    m_motor = new SparkMax(KickerConstants.kMotorPort, MotorType.kBrushless);
 }

 @Override 
 public void setVoltage(double volts) {
    m_motor.setVoltage(volts);
 }

  @Override 
  public void updateInputs(KickerIOInputs inputs) {
    inputs.appliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.currentAmps = m_motor.getOutputCurrent();
  }
}
