package frc.robot.subsystems.amp;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

public class AmpIOFalcon implements AmpIO {
  private TalonFX m_motor;
  private StatusSignal<Double> m_velocitySignal;
  private StatusSignal<Double> m_currentSignal;
  private StatusSignal<Double> m_voltageSignal;
  private StatusSignal<Double> m_angleSignal;

  public AmpIOFalcon(int port) {
    m_motor = new TalonFX(port);

    m_velocitySignal = m_motor.getVelocity();
    m_currentSignal = m_motor.getSupplyCurrent();
    m_voltageSignal = m_motor.getMotorVoltage();
    m_angleSignal = m_motor.getPosition();
  }

  @Override
  public void updateInputs(AmpInputs inputs) {
    BaseStatusSignal.refreshAll(m_velocitySignal, m_currentSignal, m_voltageSignal);

    inputs.curVelocity = m_velocitySignal.getValueAsDouble();
    inputs.curAmps = m_currentSignal.getValueAsDouble();
    inputs.curVoltage = m_voltageSignal.getValueAsDouble();
    inputs.curAngle = m_angleSignal.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}
