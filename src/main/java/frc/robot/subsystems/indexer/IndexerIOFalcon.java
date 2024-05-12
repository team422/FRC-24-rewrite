package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOFalcon implements IndexerIO {
  private TalonFX m_indexerMotor;
  private DigitalInput m_beamBreakOne;
  private DigitalInput m_beamBreakTwo;

  private StatusSignal<Double> m_curVelocity;
  private StatusSignal<Double> m_curVoltage;
  private StatusSignal<Double> m_curAmps;

  public IndexerIOFalcon(int motorPort, int beamBreakOnePort, int beamBreakTwoPort) {
    m_indexerMotor = new TalonFX(motorPort);
    m_beamBreakOne = new DigitalInput(beamBreakOnePort);
    m_beamBreakTwo = new DigitalInput(beamBreakTwoPort);

    m_curVelocity = m_indexerMotor.getVelocity();
    m_curVoltage = m_indexerMotor.getMotorVoltage();
    m_curAmps = m_indexerMotor.getSupplyCurrent();
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    BaseStatusSignal.refreshAll(m_curVelocity, m_curVoltage, m_curAmps);

    inputs.curVelocity = Units.rotationsToRadians(m_curVelocity.getValue());
    inputs.curVoltage = m_curVoltage.getValue();
    inputs.curAmps = m_curAmps.getValue();
    inputs.beamBreakOneBroken = m_beamBreakOne.get();
    inputs.beamBreakTwoBroken = m_beamBreakTwo.get();
  }

  @Override
  public void setVoltage(double voltage) {
    m_indexerMotor.setVoltage(voltage);
  }
}
