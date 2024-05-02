package frc.robot.subsystems.shooter.flywheel;

import frc.lib.advantagekit.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO extends LoggedIO<FlywheelIO.FlywheelInputs> {
  @AutoLog
  public static class FlywheelInputs {
    public double[] angularVelocity;
    public double[] linearVelocity;
    public double[] curVoltage;
    public double[] curAmps;
  }

  public void setVoltage(double leftVoltage, double rightVoltage);
}
