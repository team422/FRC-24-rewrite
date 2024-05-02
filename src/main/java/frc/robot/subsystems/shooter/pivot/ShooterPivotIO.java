package frc.robot.subsystems.shooter.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.advantagekit.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterPivotIO extends LoggedIO<ShooterPivotIO.ShooterPivotInputs> {
  @AutoLog
  public static class ShooterPivotInputs {
    public double curAngle;
    public double[] curVelocity;
    public double[] curVoltage;
    public double[] curAmps;
  }

  public void setVoltage(double voltage);

  public Rotation2d getCurrentAngle();
}
