package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOFalcon implements IndexerIO {
  private TalonFX m_kickerMotor;
  private TalonFX m_feederMotor;
  private DigitalInput m_beamBreakOne;
  private DigitalInput m_beamBreakTwo;

  private StatusSignal<Double> m_curKickerVelocity;
  private StatusSignal<Double> m_curKickerVoltage;
  private StatusSignal<Double> m_curKickerAmps;
  private StatusSignal<Double> m_curFeederVelocity;
  private StatusSignal<Double> m_curFeederVoltage;
  private StatusSignal<Double> m_curFeederAmps;

  public IndexerIOFalcon(
      int kickerPort, int feederPort, int beamBreakOnePort, int beamBreakTwoPort) {
    m_kickerMotor = new TalonFX(kickerPort);
    m_feederMotor = new TalonFX(feederPort);
    m_beamBreakOne = new DigitalInput(beamBreakOnePort);
    m_beamBreakTwo = new DigitalInput(beamBreakTwoPort);

    m_curKickerVelocity = m_kickerMotor.getVelocity();
    m_curKickerVoltage = m_kickerMotor.getMotorVoltage();
    m_curKickerAmps = m_kickerMotor.getSupplyCurrent();

    m_curFeederVelocity = m_feederMotor.getVelocity();
    m_curFeederVoltage = m_feederMotor.getMotorVoltage();
    m_curFeederAmps = m_feederMotor.getSupplyCurrent();
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    BaseStatusSignal.refreshAll(
        m_curKickerVelocity,
        m_curKickerVoltage,
        m_curKickerAmps,
        m_curFeederVelocity,
        m_curFeederVoltage,
        m_curFeederAmps);

    inputs.curKickerVelocity = Units.rotationsToRadians(m_curKickerVelocity.getValue());
    inputs.curKickerVoltage = m_curKickerVoltage.getValue();
    inputs.curKickerAmps = m_curKickerAmps.getValue();

    inputs.curFeederVelocity = Units.rotationsToRadians(m_curFeederVelocity.getValue());
    inputs.curFeederVoltage = m_curFeederVoltage.getValue();
    inputs.curFeederAmps = m_curFeederAmps.getValue();

    inputs.beamBreakOneBroken = m_beamBreakOne.get();
    inputs.beamBreakTwoBroken = m_beamBreakTwo.get();
  }

  @Override
  public void setKickerVoltage(double voltage) {
    m_kickerMotor.setVoltage(voltage);
  }

  @Override
  public void setFeederVoltage(double voltage) {
    m_feederMotor.setVoltage(voltage);
  }
}
