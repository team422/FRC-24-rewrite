package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.advantagekit.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO extends LoggedIO<IntakePivotIO.IntakePivotInputs> {
  @AutoLog
  public static class IntakePivotInputs {
    public double curAngle;
    public double curVelocity;
    public double curVoltage;
    public double curAmps;
  }

  public void setVoltage(double voltage);

  public Rotation2d getCurrentAngle();
}
