package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class RobotState {

  private static RobotState instance = null;

  // subsystems
  private Drive m_drive;
  private Shooter m_shooter;
  private Indexer m_indexer;

  // mechanism
  private final boolean kMechanismEnabled = true;
  Mechanism2d m_mechanism;

  MechanismLigament2d m_shooterPivotMechLower;
  MechanismLigament2d m_shooterPivotMechUpper;

  // advantagescope components
  private final boolean kComponentsEnabled = true;
  private final Translation3d kShooterZeroTranslation = new Translation3d(0.017, 0.0, 0.415);

  private RobotState(Drive drive, Shooter shooter, Indexer indexer) {
    // subsystems
    m_drive = drive;
    m_shooter = shooter;

    // mechanism set up
    m_mechanism = new Mechanism2d(1.5, 1);
    MechanismRoot2d m_root = m_mechanism.getRoot("shooter", 1.0, 0.5);

    // have as two ligaments so it can rotate around center
    m_shooterPivotMechLower =
        m_root.append(
            new MechanismLigament2d(
                "pivotLower",
                0.5,
                m_shooter.getPivotAngle().getDegrees() + 180,
                6,
                new Color8Bit(Color.kPurple)));

    m_shooterPivotMechUpper =
        m_root.append(
            new MechanismLigament2d(
                "pivotUpper",
                0.5,
                m_shooter.getPivotAngle().getDegrees(),
                6,
                new Color8Bit(Color.kPurple)));

    // post to dashboard
    SmartDashboard.putData("Mech2d", m_mechanism);
  }

  public static RobotState startInstance(Drive drive, Shooter shooter, Indexer indexer) {
    if (instance == null) {
      instance = new RobotState(drive, shooter, indexer);
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
    m_shooterPivotMechLower.setAngle(m_shooter.getPivotAngle().getDegrees() + 180);
    m_shooterPivotMechUpper.setAngle(m_shooter.getPivotAngle().getDegrees());

    Logger.recordOutput("Mechanism", m_mechanism);
  }

  public void updateComponents() {
    Pose3d shooterPose =
        new Pose3d(
            kShooterZeroTranslation, new Rotation3d(0, -m_shooter.getPivotAngle().getRadians(), 0));

    Logger.recordOutput("Components/ShooterPose", shooterPose);
    Logger.recordOutput("Components/ZeroPose", new Pose3d()); // for tuning config
  }

  public void updateRobotState() {
    if (kMechanismEnabled) {
      updateMechanism();
    }

    if (kComponentsEnabled) {
      updateComponents();
    }
  }
}
