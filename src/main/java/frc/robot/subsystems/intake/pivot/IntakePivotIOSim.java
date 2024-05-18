package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.IntakeConstants;

public class IntakePivotIOSim implements IntakePivotIO {
  private SingleJointedArmSim m_sim;
  private double m_voltage;

  public IntakePivotIOSim() {
    DCMotor gearbox = DCMotor.getNeo550(1);
    double gearing = 24.2391;
    double armLength = 0.21;
    double jKgMetersSquared = SingleJointedArmSim.estimateMOI(armLength, 4);
    double minAngle = IntakeConstants.kMinAngle.getDegrees();
    double maxAngle = IntakeConstants.kMaxAngle.getDegrees();
    double startingAngle = IntakeConstants.kHomeAngle.getDegrees();

    m_sim =
        new SingleJointedArmSim(
            gearbox,
            gearing,
            jKgMetersSquared,
            armLength,
            minAngle,
            maxAngle,
            false,
            startingAngle);

    m_voltage = 0.0;
  }

  @Override
  public void updateInputs(IntakePivotInputs inputs) {
    m_sim.update(0.02);

    inputs.curAngle = getCurrentAngle().getRadians();
    inputs.curVoltage = m_voltage;
    inputs.curVelocity = m_sim.getVelocityRadPerSec();
    inputs.curAmps = m_sim.getCurrentDrawAmps();
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
