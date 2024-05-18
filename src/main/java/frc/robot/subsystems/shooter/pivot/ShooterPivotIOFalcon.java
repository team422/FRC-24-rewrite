package frc.robot.subsystems.shooter.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ShooterConstants;

public class ShooterPivotIOFalcon implements ShooterPivotIO {
  private TalonFX m_leader;
  private TalonFX m_follower;
  private DutyCycleEncoder m_absoluteEncoder;

  private StatusSignal<Double> m_leaderVoltageSignal;
  private StatusSignal<Double> m_leaderCurrentSignal;
  private StatusSignal<Double> m_leaderVelocitySignal;
  private StatusSignal<Double> m_followerVoltageSignal;
  private StatusSignal<Double> m_followerCurrentSignal;
  private StatusSignal<Double> m_followerVelocitySignal;

  public ShooterPivotIOFalcon(int leaderPort, int followerPort, int absoluteEncoderPort) {
    m_leader = new TalonFX(leaderPort);
    m_follower = new TalonFX(followerPort);
    m_absoluteEncoder = new DutyCycleEncoder(absoluteEncoderPort);

    // follow leader
    m_follower.setControl(new Follower(leaderPort, false));

    // status signals
    m_leaderVoltageSignal = m_leader.getMotorVoltage();
    m_leaderCurrentSignal = m_leader.getSupplyCurrent();
    m_leaderVelocitySignal = m_leader.getVelocity();
    m_followerVoltageSignal = m_follower.getMotorVoltage();
    m_followerCurrentSignal = m_follower.getSupplyCurrent();
    m_followerVelocitySignal = m_follower.getVelocity();

    m_absoluteEncoder.setPositionOffset(ShooterConstants.kPivotOffset.getRotations());
  }

  @Override
  public void updateInputs(ShooterPivotInputs inputs) {
    BaseStatusSignal.refreshAll(
        m_leaderVoltageSignal,
        m_leaderCurrentSignal,
        m_leaderVelocitySignal,
        m_followerVoltageSignal,
        m_followerCurrentSignal,
        m_followerVelocitySignal);

    inputs.curAngle = getCurrentAngle().getRadians();
    inputs.curVelocity =
        new double[] {
          m_leaderVelocitySignal.getValueAsDouble(), m_followerVelocitySignal.getValueAsDouble()
        };
    inputs.curVoltage =
        new double[] {
          m_leaderVoltageSignal.getValueAsDouble(), m_followerVoltageSignal.getValueAsDouble()
        };
    inputs.curAmps =
        new double[] {
          m_leaderCurrentSignal.getValueAsDouble(), m_followerCurrentSignal.getValueAsDouble()
        };
  }

  @Override
  public void setVoltage(double voltage) {
    m_leader.setVoltage(voltage);
  }

  @Override
  public Rotation2d getCurrentAngle() {
    return Rotation2d.fromRotations(m_absoluteEncoder.get());
  }
}
