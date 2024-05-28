package frc.robot.subsystems.intake.pivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeConstants;

public class IntakePivotIONeo implements IntakePivotIO {
  private CANSparkMax m_motor;
  private AbsoluteEncoder m_encoder;

  public IntakePivotIONeo(int port) {
    m_motor = new CANSparkMax(port, CANSparkMax.MotorType.kBrushless);
    m_encoder = m_motor.getAbsoluteEncoder();
    m_encoder.setPositionConversionFactor(IntakeConstants.kPivotGearRatio);

    m_encoder.setZeroOffset(IntakeConstants.kPivotOffset.getRadians());
  }

  @Override
  public void updateInputs(IntakePivotInputs inputs) {
    inputs.curAngle = Units.rotationsToRadians(m_encoder.getPosition());
    inputs.curVelocity =
        Units.rotationsPerMinuteToRadiansPerSecond(m_motor.getEncoder().getVelocity());
    inputs.curVoltage = m_motor.getAppliedOutput();
    inputs.curAmps = m_motor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}
