package frc.robot.subsystems.intake.pivot;

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
    double minAngle = IntakeConstants.kMinAngle.getRadians();
    double maxAngle = IntakeConstants.kMaxAngle.getRadians();
    double startingAngle = IntakeConstants.kHomeAngle.getRadians();

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

    inputs.curAngle = m_sim.getAngleRads();
    inputs.curVoltage = m_voltage;
    inputs.curVelocity = m_sim.getVelocityRadPerSec();
    inputs.curAmps = m_sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double voltage) {
    m_sim.setInputVoltage(voltage);
    m_voltage = voltage;
  }
}
