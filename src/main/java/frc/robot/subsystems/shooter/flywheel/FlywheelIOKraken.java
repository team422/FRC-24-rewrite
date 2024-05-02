package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterConstants;

public class FlywheelIOKraken implements FlywheelIO {
  private TalonFX m_leftMotor;
  private TalonFX m_rightMotor;

  private StatusSignal<Double> m_leftVoltageSignal;
  private StatusSignal<Double> m_leftCurrentSignal;
  private StatusSignal<Double> m_leftVelocitySignal;
  private StatusSignal<Double> m_rightVoltageSignal;
  private StatusSignal<Double> m_rightCurrentSignal;
  private StatusSignal<Double> m_rightVelocitySignal;

  public FlywheelIOKraken(int leftPort, int rightPort) {
    m_leftMotor = new TalonFX(leftPort);
    m_rightMotor = new TalonFX(rightPort);

    // status signals
    m_leftVoltageSignal = m_leftMotor.getMotorVoltage();
    m_leftCurrentSignal = m_leftMotor.getSupplyCurrent();
    m_leftVelocitySignal = m_leftMotor.getVelocity();
    m_rightVoltageSignal = m_rightMotor.getMotorVoltage();
    m_rightCurrentSignal = m_rightMotor.getSupplyCurrent();
    m_rightVelocitySignal = m_rightMotor.getVelocity();
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    BaseStatusSignal.refreshAll(
        m_leftVoltageSignal,
        m_leftCurrentSignal,
        m_leftVelocitySignal,
        m_rightVoltageSignal,
        m_rightCurrentSignal,
        m_rightVelocitySignal);

    inputs.angularVelocity =
        new double[] {
          Units.rotationsToRadians(m_leftVelocitySignal.getValueAsDouble()),
          Units.rotationsToRadians(m_rightVelocitySignal.getValueAsDouble())
        };

    inputs.linearVelocity =
        new double[] {
          rotationsPerSecondToMeterPerSecond(m_leftVelocitySignal.getValueAsDouble()),
          rotationsPerSecondToMeterPerSecond(m_rightVelocitySignal.getValueAsDouble())
        };

    inputs.curVoltage =
        new double[] {
          m_leftVoltageSignal.getValueAsDouble(), m_rightVoltageSignal.getValueAsDouble()
        };

    inputs.curAmps =
        new double[] {
          m_leftCurrentSignal.getValueAsDouble(), m_rightCurrentSignal.getValueAsDouble()
        };
  }

  private double rotationsPerSecondToMeterPerSecond(double rotationsPerSecond) {
    double radius = ShooterConstants.kFlywheelDiameter / 2;
    double radPerSec = Units.rotationsToRadians(rotationsPerSecond);
    return radius * radPerSec;
  }

  @Override
  public void setVoltage(double leftVoltage, double rightVoltage) {
    m_leftMotor.setVoltage(leftVoltage);
    m_rightMotor.setVoltage(rightVoltage);
  }
}
