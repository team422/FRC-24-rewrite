package frc.robot.subsystems.shooter.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ShooterConstants;

public class ShooterPivotIOSim implements ShooterPivotIO {
  private SingleJointedArmSim m_sim;
  private double m_voltage;

  public ShooterPivotIOSim() {
    m_sim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(2),
            46.722,
            10,
            .24,
            ShooterConstants.kMinAngle.getRadians(),
            ShooterConstants.kMaxAngle.getRadians(),
            false,
            ShooterConstants.kHomeAngle.getRadians());
    m_voltage = 0.0;
  }

  @Override
  public void updateInputs(ShooterPivotInputs inputs) {
    m_sim.update(0.02);

    // distinction between left and right doesnt matter in sim
    inputs.curAngle = getCurrentAngle().getRadians();
    inputs.curVoltage = new double[] {m_voltage};
    inputs.curVelocity = new double[] {m_sim.getVelocityRadPerSec()};
    inputs.curAmps = new double[] {m_sim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double voltage) {
    m_sim.setInputVoltage(voltage);
    m_voltage = voltage;
  }

  @Override
  public Rotation2d getCurrentAngle() {
    return Rotation2d.fromRadians(m_sim.getAngleRads());
  }
}
