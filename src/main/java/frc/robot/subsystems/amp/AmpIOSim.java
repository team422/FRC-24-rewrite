package frc.robot.subsystems.amp;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.AmpConstants;

public class AmpIOSim implements AmpIO {
  private SingleJointedArmSim m_sim;
  private double m_voltage;

  public AmpIOSim() {
    DCMotor gearbox = DCMotor.getFalcon500(1);
    double gearing = 20;
    double armLength = 0.4572;
    double jKgMetersSquared = SingleJointedArmSim.estimateMOI(armLength, 0.91);

    double minAngle = AmpConstants.kMinAngle.getRadians();
    double maxAngle = AmpConstants.kMaxAngle.getRadians();
    double startingAngle = AmpConstants.kHomeAngle.getRadians();

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
  public void updateInputs(AmpInputs inputs) {
    m_sim.update(0.02);

    inputs.curAngle = Units.radiansToRotations(m_sim.getAngleRads());
    inputs.curVoltage = m_voltage;
    inputs.curVelocity = Units.radiansToRotations(m_sim.getVelocityRadPerSec());
    inputs.curAmps = m_sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double voltage) {
    m_sim.setInputVoltage(voltage);
    m_voltage = voltage;
  }
}
