package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
  public double getDriveForward();

  public double getDriveLeft();

  public double getDriveRotation();

  public Trigger revShooterLow();

  public Trigger revShooterMid();

  public Trigger revShooterHigh();

  public Trigger runKicker();

  public Trigger deployIntake();
}
