package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;

public class RobotState {
  private static RobotState instance = null;

  // subsystems
  private Drive m_drive;
  private Shooter m_shooter;

  // mechanism
  private final boolean kMechanismEnabled = true;
  MechanismLigament2d m_shooterPivotMech;

  private RobotState(Drive drive, Shooter shooter) {
    // subsystems
    m_drive = drive;
    m_shooter = shooter;

    // mechanism set up
    Mechanism2d m_mechanism = new Mechanism2d(3, 3);
    MechanismRoot2d m_root = m_mechanism.getRoot("shooter", 3, 1);
    m_shooterPivotMech =
        m_root.append(
            new MechanismLigament2d(
                "pivot",
                1,
                m_shooter.getPivotAngle().getDegrees(),
                6,
                new Color8Bit(Color.kPurple)));

    // post to dashboard
    SmartDashboard.putData("Mech2d", m_mechanism);
  }

  public static RobotState startInstance(Drive drive, Shooter shooter) {
    if (instance == null) {
      instance = new RobotState(drive, shooter);
    } else {
      throw new IllegalStateException("RobotState instance already started");
    }
    return instance;
  }

  public static RobotState getInstance() {
    if (instance == null) {
      throw new IllegalStateException("RobotState instance not started");
    }
    return instance;
  }

  public void updateMechanism() {
    m_shooterPivotMech.setAngle(m_shooter.getPivotAngle().getDegrees());
  }

  public void updateRobotState() {
    if (kMechanismEnabled) {
      updateMechanism();
    }
  }
}
