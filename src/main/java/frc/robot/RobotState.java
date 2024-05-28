package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class RobotState {

  private static RobotState instance = null;

  // subsystems
  private Drive m_drive;
  private Shooter m_shooter;
  private Indexer m_indexer;
  private Intake m_intake;

  // mechanism
  private final boolean kMechanismEnabled = true;
  Mechanism2d m_mechanism;

  MechanismLigament2d m_shooterPivotMechLower;
  MechanismLigament2d m_shooterPivotMechUpper;
  MechanismLigament2d m_intakePivotMech;

  // advantagescope components
  private final boolean kComponentsEnabled = true;
  private final Pose3d kShooterZeroPose =
      new Pose3d(new Translation3d(0.017, 0.0, 0.415), new Rotation3d());
  private final Pose3d kIntakeZeroPose =
      new Pose3d(
          new Translation3d(-0.269, -0.01, 0.2428),
          new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(170), 0.0));

  private RobotState(Drive drive, Shooter shooter, Indexer indexer, Intake intake) {
    // subsystems
    m_drive = drive;
    m_shooter = shooter;
    m_indexer = indexer;
    m_intake = intake;

    // mechanism set up
    m_mechanism = new Mechanism2d(1.75, 1);
    MechanismRoot2d m_root = m_mechanism.getRoot("shooter", 1.25, 0.5);

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

    MechanismRoot2d intakeRoot = m_mechanism.getRoot("intake", 0.7, 0.25);
    m_intakePivotMech =
        intakeRoot.append(
            new MechanismLigament2d(
                "pivot",
                0.65,
                185 - m_intake.getPivotAngle().getDegrees(),
                6,
                new Color8Bit(Color.kBlue)));

    // post to dashboard
    SmartDashboard.putData("Mech2d", m_mechanism);
  }

  public static RobotState startInstance(
      Drive drive, Shooter shooter, Indexer indexer, Intake intake) {
    if (instance == null) {
      instance = new RobotState(drive, shooter, indexer, intake);
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
    m_intakePivotMech.setAngle(185 - m_intake.getPivotAngle().getDegrees());

    Logger.recordOutput("Mechanism", m_mechanism);
  }

  public void updateComponents() {
    Pose3d shooterPose =
        new Pose3d(
            kShooterZeroPose.getTranslation(),
            kShooterZeroPose
                .getRotation()
                .rotateBy(new Rotation3d(0, -m_shooter.getPivotAngle().getRadians(), 0)));
    Pose3d intakePose =
        new Pose3d(
            kIntakeZeroPose.getTranslation(),
            kIntakeZeroPose
                .getRotation()
                .rotateBy(new Rotation3d(0, m_intake.getPivotAngle().getRadians(), 0)));

    Logger.recordOutput("Components/ShooterPose", shooterPose);
    Logger.recordOutput("Components/IntakePose", intakePose);
    Logger.recordOutput("Components/Poses", new Pose3d[] {shooterPose, intakePose});
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
