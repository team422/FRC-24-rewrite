package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;

public interface IndexerIO extends LoggedIO<IndexerIO.IndexerInputs> {
    @AutoLog
    public static class IndexerInputs {
        public double curVelocity;
        public double curVoltage;
        public double curAmps;
        public boolean beamBreakOneBroken;
        public boolean beamBreakTwoBroken;
    }

    public void setVoltage(double voltage);

}
