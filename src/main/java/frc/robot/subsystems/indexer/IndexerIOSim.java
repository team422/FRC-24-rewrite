package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.Constants.Ports;

public class IndexerIOSim implements IndexerIO {
  private DCMotorSim m_kickerSim;
  private DCMotorSim m_feederSim;
  private double m_curKickerVoltage;
  private double m_curFeederVoltage;
  private DIOSim m_beamBreakOne;
  private DIOSim m_beamBreakTwo;

  public IndexerIOSim() {
    double simGearing = 1.0;
    double simJkGMetersSquared = 0.01;
    m_kickerSim = new DCMotorSim(DCMotor.getFalcon500(1), simGearing, simJkGMetersSquared);
    m_feederSim = new DCMotorSim(DCMotor.getFalcon500(1), simGearing, simJkGMetersSquared);

    m_beamBreakOne = new DIOSim(Ports.kIndexerBeamBreakOne);
    m_beamBreakTwo = new DIOSim(Ports.kIndexerBeamBreakTwo);

    m_curKickerVoltage = 0.0;
    m_curFeederVoltage = 0.0;
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    m_kickerSim.update(0.02);
    m_feederSim.update(0.02);

    inputs.curKickerVoltage = m_curKickerVoltage;
    inputs.curKickerVelocity = m_kickerSim.getAngularVelocityRadPerSec();
    inputs.curKickerAmps = m_kickerSim.getCurrentDrawAmps();

    inputs.curFeederVoltage = m_curFeederVoltage;
    inputs.curFeederVelocity = m_feederSim.getAngularVelocityRadPerSec();
    inputs.curFeederAmps = m_feederSim.getCurrentDrawAmps();

    inputs.beamBreakOneBroken = m_beamBreakOne.getValue();
    inputs.beamBreakTwoBroken = m_beamBreakTwo.getValue();
  }

  @Override
  public void setKickerVoltage(double voltage) {
    m_kickerSim.setInputVoltage(voltage);
    m_curKickerVoltage = voltage;
  }

  @Override
  public void setFeederVoltage(double voltage) {
    m_feederSim.setInputVoltage(voltage);
    m_curFeederVoltage = voltage;
  }
}
