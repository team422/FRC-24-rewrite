package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterConstants;

public class FlywheelIOSim implements FlywheelIO {
  private FlywheelSim m_leftSim;
  private FlywheelSim m_rightSim;
  private double m_leftVoltage;
  private double m_rightVoltage;

  public FlywheelIOSim() {
    m_leftSim = new FlywheelSim(DCMotor.getKrakenX60(1), 1 / 1.5, 1.0);
    m_rightSim = new FlywheelSim(DCMotor.getKrakenX60(1), 1 / 1.5, 1.0);
    m_leftVoltage = 0.0;
    m_rightVoltage = 0.0;
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    m_leftSim.update(0.02);
    m_rightSim.update(0.02);

    inputs.curVoltage = new double[] {m_leftVoltage, m_rightVoltage};

    inputs.angularVelocityRad =
        new double[] {
          m_leftSim.getAngularVelocityRadPerSec(), m_leftSim.getAngularVelocityRadPerSec()
        };
    
    inputs.angularVelocityRPS =
        new double[] {
          m_leftSim.getAngularVelocityRPM() / 60.0, m_rightSim.getAngularVelocityRPM() / 60.0
        };

    inputs.linearVelocity =
        new double[] {
          m_leftSim.getAngularVelocityRadPerSec() * ShooterConstants.kFlywheelDiameter / 2.0,
          m_rightSim.getAngularVelocityRadPerSec() * ShooterConstants.kFlywheelDiameter / 2.0
        };

    inputs.curAmps = new double[] {m_leftSim.getCurrentDrawAmps(), m_rightSim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double leftVoltage, double rightVoltage) {
    m_leftSim.setInputVoltage(leftVoltage);
    m_rightSim.setInputVoltage(rightVoltage);
    m_leftVoltage = leftVoltage;
    m_rightVoltage = rightVoltage;
  }
}
