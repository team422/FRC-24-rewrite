package frc.robot.subsystems.amp;

import frc.lib.advantagekit.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

public interface AmpIO extends LoggedIO<AmpIO.AmpInputs> {
  @AutoLog
  public static class AmpInputs {
    public double curVoltage;
    public double curAmps;
    public double curVelocity;
    public double curAngle;
  }

  public void setVoltage(double voltage);
}
