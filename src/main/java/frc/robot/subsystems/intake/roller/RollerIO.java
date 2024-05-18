package frc.robot.subsystems.intake.roller;

import frc.lib.advantagekit.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO extends LoggedIO<RollerIO.RollerInputs> {
  @AutoLog
  public static class RollerInputs {
    public double curVelocity;
    public double curVoltage;
    public double curAmps;
  }

  public void setVoltage(double voltage);
}
