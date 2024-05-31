package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
  private AddressableLED m_strip;
  private AddressableLEDBuffer m_buffer;
  private LedState m_state = LedState.OFF;

  public static enum LedState {
    OFF,
    AUTONOMOUS,
    TELEOP,
    SHOOTER_NOT_READY,
    SHOOTER_READY,
    DISABLED_NOT_READY,
    DISABLED_READY
  }

  public Led(int port, int length) {
    m_strip = new AddressableLED(port);
    m_buffer = new AddressableLEDBuffer(length);
    m_strip.setLength(length);
    m_strip.start();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LED/State", m_state.toString());
    Logger.recordOutput("LED/Color", m_buffer.getLED(0).toString());
  }

  public void setSolidColor(Color color) {
    for (int i = 0; i < m_buffer.getLength(); i++) {
      m_buffer.setLED(i, color);
    }
    m_strip.setData(m_buffer);
  }

  public void setState(LedState state) {
    if (state == m_state) {
      return;
    }
    switch (state) {
      case OFF:
        setSolidColor(Color.kBlack);
        break;
      case SHOOTER_READY:
        setSolidColor(Color.kYellow);
        break;
      case AUTONOMOUS:
        setSolidColor(Color.kBlue);
        break;
      case TELEOP:
        setSolidColor(Color.kMagenta);
        break;
      case DISABLED_NOT_READY:
        setSolidColor(Color.kRed);
        break;
      case SHOOTER_NOT_READY:
      case DISABLED_READY:
        setSolidColor(Color.kGreen);
        break;
    }
    m_state = state;
  }
}
