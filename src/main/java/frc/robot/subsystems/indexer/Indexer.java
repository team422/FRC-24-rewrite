package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerIO m_io;
  public final IndexerInputsAutoLogged m_inputs;

  public Indexer(IndexerIO io) {
    m_io = io;
    m_inputs = new IndexerInputsAutoLogged();
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);

    Logger.processInputs("Indexer", m_inputs);
  }

  public void setKickerVoltage(double voltage) {
    m_io.setKickerVoltage(voltage);
  }

  public void setFeederVoltage(double voltage) {
    m_io.setFeederVoltage(voltage);
  }

  public Command runKicker(double voltage) {
    return Commands.runOnce(() -> m_io.setKickerVoltage(voltage));
  }

  public Command runFeeder(double voltage) {
    return Commands.runOnce(() -> m_io.setFeederVoltage(voltage));
  }
}
