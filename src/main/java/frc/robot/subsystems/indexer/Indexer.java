package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private IndexerIO m_io;

    public Indexer(IndexerIO io) {
        m_io = io;
    }

    public Command runIndexer(double voltage) {
        return Commands.runOnce(() -> m_io.setVoltage(voltage));
    }
}
