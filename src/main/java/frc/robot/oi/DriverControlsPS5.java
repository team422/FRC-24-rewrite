package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsPS5 implements DriverControls {
  private CommandPS5Controller m_controller;

  public DriverControlsPS5(int port) {
    m_controller = new CommandPS5Controller(port);
  }

  @Override
  public double getDriveForward() {
    return -m_controller.getLeftY();
  }

  @Override
  public double getDriveLeft() {
    return -m_controller.getLeftX();
  }

  @Override
  public double getDriveRotation() {
    return -m_controller.getRightX();
  }

  @Override
  public Trigger revShooterLow() {
    return m_controller.circle();
  }

  @Override
  public Trigger revShooterMid() {
    return m_controller.L2();
  }

  @Override
  public Trigger revShooterHigh() {
    return m_controller.square();
  }

  @Override
  public Trigger runKicker() {
    return m_controller.R1();
  }

  @Override
  public Trigger deployIntake() {
    return m_controller.R2();
  }
}
