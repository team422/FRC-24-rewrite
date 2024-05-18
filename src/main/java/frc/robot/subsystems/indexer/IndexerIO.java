package frc.robot.subsystems.indexer;

import frc.lib.advantagekit.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO extends LoggedIO<IndexerIO.IndexerInputs> {
  @AutoLog
  public static class IndexerInputs {
    public double curKickerVelocity;
    public double curKickerVoltage;
    public double curKickerAmps;
    public double curFeederVelocity;
    public double curFeederVoltage;
    public double curFeederAmps;
    public boolean beamBreakOneBroken;
    public boolean beamBreakTwoBroken;
  }

  public void setKickerVoltage(double voltage);

  public void setFeederVoltage(double voltage);
}
