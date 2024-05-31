package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;

public class RollerIOKraken implements RollerIO {
  private TalonFX m_motor;
  private StatusSignal<Double> m_voltageSignal;
  private StatusSignal<Double> m_currentSignal;
  private StatusSignal<Double> m_velocitySignal;

  public RollerIOKraken(int port) {
    m_motor = new TalonFX(port);
    m_voltageSignal = m_motor.getMotorVoltage();
    m_currentSignal = m_motor.getSupplyCurrent();
    m_velocitySignal = m_motor.getVelocity();
  }

  @Override
  public void updateInputs(RollerInputs inputs) {
    BaseStatusSignal.refreshAll(m_voltageSignal, m_currentSignal, m_velocitySignal);

    inputs.curVoltage = m_voltageSignal.getValueAsDouble();
    inputs.curAmps = m_currentSignal.getValueAsDouble();
    inputs.curVelocity = Units.rotationsToRadians(m_velocitySignal.getValueAsDouble());
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setControl(new VoltageOut(voltage));
  }
}
