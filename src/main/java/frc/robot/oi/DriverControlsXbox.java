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
    return m_controller.getLeftY();
  }

  @Override
  public double getDriveLeft() {
    return m_controller.getLeftX();
  }

  @Override
  public double getDriveRotation() {
    return m_controller.getRightX();
  }

  @Override
  public Trigger testShooter() {
    return m_controller.button(1);
  }

  @Override
  public Trigger testKicker() {
    return m_controller.button(2);
  }

  @Override
  public Trigger testFeeder() {
    return m_controller.button(3);
  }
}
