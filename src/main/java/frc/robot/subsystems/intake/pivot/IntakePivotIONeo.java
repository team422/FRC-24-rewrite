package frc.robot.subsystems.intake.pivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeConstants;

public class IntakePivotIONeo implements IntakePivotIO {
  private CANSparkMax m_motor;
  private AbsoluteEncoder m_absEncoder;
  private RelativeEncoder m_relEncoder;

  public IntakePivotIONeo(int port) {
    m_motor = new CANSparkMax(port, CANSparkMax.MotorType.kBrushless);
    m_absEncoder = m_motor.getAbsoluteEncoder();

    // do not ask me how, but the gear ratio causes the position unit to be in radians
    // all future times that units are used for this encoder are in radians
    m_absEncoder.setPositionConversionFactor(IntakeConstants.kPivotGearRatio);
    m_absEncoder.setZeroOffset(IntakeConstants.kPivotOffset.getRadians());

    m_relEncoder = m_motor.getEncoder();
  }

  @Override
  public void updateInputs(IntakePivotInputs inputs) {
    inputs.curAngle = m_absEncoder.getPosition();

    inputs.curVelocity = Units.rotationsPerMinuteToRadiansPerSecond(m_relEncoder.getVelocity());
    inputs.curVoltage = m_motor.getAppliedOutput();
    inputs.curAmps = m_motor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}
