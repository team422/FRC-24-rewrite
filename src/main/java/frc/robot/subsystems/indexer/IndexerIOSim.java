package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.Constants.Ports;

public class IndexerIOSim implements IndexerIO {
    private DCMotorSim m_motorSim;
    private double m_curVoltage;
    private DIOSim m_beamBreakOne;
    private DIOSim m_beamBreakTwo;

    public IndexerIOSim() {
        double simGearing = 1.0;
        double simJkGMetersSquared = 1.0;
        m_motorSim = new DCMotorSim(
            DCMotor.getFalcon500(1),
            simGearing,
            simJkGMetersSquared
        );

        m_beamBreakOne = new DIOSim(Ports.kIndexerBeamBreakOne);
        m_beamBreakTwo = new DIOSim(Ports.kIndexerBeamBreakTwo);
    }

    @Override
    public void updateInputs(IndexerInputs inputs) {
        m_motorSim.update(0.02);

        inputs.curVoltage = m_curVoltage;
        inputs.curVelocity = m_motorSim.getAngularVelocityRadPerSec();
        inputs.curAmps = m_motorSim.getCurrentDrawAmps();
        inputs.beamBreakOneBroken = m_beamBreakOne.getValue();
        inputs.beamBreakTwoBroken = m_beamBreakTwo.getValue();
    }

    @Override
    public void setVoltage(double voltage) {
        m_motorSim.setInputVoltage(voltage);
        m_curVoltage = voltage;
    }
}
