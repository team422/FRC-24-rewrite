package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsXbox implements DriverControls {
  private CommandXboxController m_controller;

  public DriverControlsXbox(int port) {
    m_controller = new CommandXboxController(port);
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
    return m_controller.b();
  }

  @Override
  public Trigger revShooterMid() {
    return m_controller.leftTrigger();
  }

  @Override
  public Trigger revShooterHigh() {
    return m_controller.x();
  }

  @Override
  public Trigger runKicker() {
    return m_controller.rightBumper();
  }

  @Override
  public Trigger deployIntake() {
    return m_controller.rightTrigger();
  }
}
