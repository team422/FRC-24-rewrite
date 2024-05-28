package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public double getDriveForward();

  public double getDriveLeft();

  public double getDriveRotation();

  public Trigger revShooter();

  public Trigger runKicker();

  public Trigger deployIntake();
}
