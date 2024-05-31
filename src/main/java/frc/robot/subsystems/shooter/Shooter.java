package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.LoggedTunableNumber;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelInputsAutoLogged;
import frc.robot.subsystems.shooter.pivot.ShooterPivotIO;
import frc.robot.subsystems.shooter.pivot.ShooterPivotInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /** Represents the desired state of the shooter, including flywheel and pivot measurements */
  public static record ShooterPosition(
      Rotation2d pivotAngle, double leftFlywheelVelocity, double rightFlywheelVelocity) {}

  private ShooterPivotIO m_pivotIO;
  private FlywheelIO m_flywheelIO;

  public final ShooterPivotInputsAutoLogged m_pivotInputs;
  public final FlywheelInputsAutoLogged m_flywheelInputs;

  private ProfiledPIDController m_pivotController;
  private PIDController m_leftFlywheelController;
  private PIDController m_rightFlywheelController;

  private SimpleMotorFeedforward m_rightFlywheelFeedforward;
  private SimpleMotorFeedforward m_leftFlywheelFeedforward;

  public Shooter(
      ShooterPivotIO pivotIO,
      FlywheelIO flywheelIO,
      ProfiledPIDController pivotController,
      PIDController leftFlywheelController,
      PIDController rightFlywheelController,
      SimpleMotorFeedforward leftFlywheelFeedforward,
      SimpleMotorFeedforward rightFlywheelFeedforward) {

    m_pivotIO = pivotIO;
    m_flywheelIO = flywheelIO;

    m_pivotController = pivotController;
    m_leftFlywheelController = leftFlywheelController;
    m_rightFlywheelController = rightFlywheelController;

    m_rightFlywheelFeedforward = rightFlywheelFeedforward;
    m_leftFlywheelFeedforward = leftFlywheelFeedforward;

    m_pivotInputs = new ShooterPivotInputsAutoLogged();
    m_flywheelInputs = new FlywheelInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // update pid if changed
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_pivotController.setP(ShooterConstants.kPivotP.get());
          m_pivotController.setI(ShooterConstants.kPivotI.get());
          m_pivotController.setD(ShooterConstants.kPivotD.get());
        },
        ShooterConstants.kPivotP,
        ShooterConstants.kPivotI,
        ShooterConstants.kPivotD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_leftFlywheelController.setP(ShooterConstants.kLeftFlywheelP.get());
          m_leftFlywheelController.setI(ShooterConstants.kLeftFlywheelI.get());
          m_leftFlywheelController.setD(ShooterConstants.kLeftFlywheelD.get());
        },
        ShooterConstants.kLeftFlywheelP,
        ShooterConstants.kLeftFlywheelI,
        ShooterConstants.kLeftFlywheelD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_rightFlywheelController.setP(ShooterConstants.kRightFlywheelP.get());
          m_rightFlywheelController.setI(ShooterConstants.kRightFlywheelI.get());
          m_rightFlywheelController.setD(ShooterConstants.kRightFlywheelD.get());
        },
        ShooterConstants.kRightFlywheelP,
        ShooterConstants.kRightFlywheelI,
        ShooterConstants.kRightFlywheelD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_rightFlywheelFeedforward =
              new SimpleMotorFeedforward(
                  ShooterConstants.kRightFlywheelKs.get(), ShooterConstants.kRightFlywheelKv.get());
        },
        ShooterConstants.kRightFlywheelKs,
        ShooterConstants.kRightFlywheelKv);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_leftFlywheelFeedforward =
              new SimpleMotorFeedforward(
                  ShooterConstants.kLeftFlywheelKs.get(), ShooterConstants.kLeftFlywheelKv.get());
        },
        ShooterConstants.kLeftFlywheelKs,
        ShooterConstants.kLeftFlywheelKv);

    m_pivotIO.updateInputs(m_pivotInputs);
    m_flywheelIO.updateInputs(m_flywheelInputs);

    double pivotPidVoltage = m_pivotController.calculate(m_pivotInputs.curAngle);
    m_pivotIO.setVoltage(pivotPidVoltage);

    double leftFlywheelPidVoltage =
        m_leftFlywheelController.calculate(m_flywheelInputs.linearVelocity[0]);
    double rightFlywheelPidVoltage =
        m_rightFlywheelController.calculate(m_flywheelInputs.linearVelocity[1]);

    double leftFlywheelFeedforwardVoltage =
        m_leftFlywheelFeedforward.calculate(m_leftFlywheelController.getSetpoint());
    double rightFlywheelFeedforwardVoltage =
        m_rightFlywheelFeedforward.calculate(m_rightFlywheelController.getSetpoint());

    m_flywheelIO.setVoltage(
        leftFlywheelPidVoltage + leftFlywheelFeedforwardVoltage,
        rightFlywheelPidVoltage + rightFlywheelFeedforwardVoltage);

    Logger.processInputs("Shooter Pivot", m_pivotInputs);
    Logger.processInputs("Shooter Flywheel", m_flywheelInputs);

    Logger.recordOutput("Shooter/Pivot/PivotPIDVoltage", pivotPidVoltage);
    Logger.recordOutput("Shooter/Pivot/PivotAngle", Units.radiansToDegrees(m_pivotInputs.curAngle));
    Logger.recordOutput(
        "Shooter/Pivot/PivotDesiredAngle",
        Units.radiansToDegrees(m_pivotController.getGoal().position));

    Logger.recordOutput("Shooter/Flywheel/LeftFlywheelPIDVoltage", leftFlywheelPidVoltage);
    Logger.recordOutput("Shooter/Flywheel/RightFlywheelPIDVoltage", rightFlywheelPidVoltage);

    Logger.recordOutput(
        "Shooter/Flywheel/LeftFlywheelFeedforwardVoltage", leftFlywheelFeedforwardVoltage);
    Logger.recordOutput(
        "Shooter/Flywheel/RightFlywheelFeedforwardVoltage", rightFlywheelFeedforwardVoltage);

    Logger.recordOutput(
        "Shooter/Flywheel/LeftFlywheelSetVoltage",
        leftFlywheelPidVoltage + leftFlywheelFeedforwardVoltage);
    Logger.recordOutput(
        "Shooter/Flywheel/RightFlywheelSetVoltage",
        rightFlywheelPidVoltage + rightFlywheelFeedforwardVoltage);

    Logger.recordOutput(
        "Shooter/Flywheel/LeftFlywheelVelocity", m_flywheelInputs.linearVelocity[0]);
    Logger.recordOutput(
        "Shooter/Flywheel/RightFlywheelVelocity", m_flywheelInputs.linearVelocity[1]);
    Logger.recordOutput(
        "Shooter/Flywheel/LeftFlywheelDesiredVelocity", m_leftFlywheelController.getSetpoint());
    Logger.recordOutput(
        "Shooter/Flywheel/RightFlywheelDesiredVelocity", m_rightFlywheelController.getSetpoint());

    // ready for match
    RobotState.getInstance().setPrematchCheckValue("ShooterPivot", m_pivotController.atGoal());
    RobotState.getInstance()
        .setPrematchCheckValue(
            "FlywheelsStill",
            Math.abs(m_flywheelInputs.linearVelocity[0]) < 0.1
                && Math.abs(m_flywheelInputs.linearVelocity[1]) < 0.1);
  }

  public void setPivotAngle(Rotation2d angle) {
    m_pivotController.setGoal(angle.getRadians());
  }

  /** Set the flywheel velocity in meters per second */
  public void setFlywheelVelocity(double leftVelocity, double rightVelocity) {
    m_leftFlywheelController.setSetpoint(leftVelocity);
    m_rightFlywheelController.setSetpoint(rightVelocity);
  }

  public Command setPivotAngleCommand(Rotation2d angle) {
    return Commands.runOnce(() -> setPivotAngle(angle));
  }

  /** Set the flywheel velocity in meters per second */
  public Command setFlywheelVelocityCommand(double leftVelocity, double rightVelocity) {
    return Commands.runOnce(() -> setFlywheelVelocity(leftVelocity, rightVelocity));
  }

  public void setShooter(ShooterPosition position) {
    setPivotAngle(position.pivotAngle());
    setFlywheelVelocity(position.leftFlywheelVelocity(), position.rightFlywheelVelocity());
  }

  public Command setShooterCommand(ShooterPosition position) {
    return Commands.runOnce(() -> setShooter(position));
  }

  public Rotation2d getPivotAngle() {
    return Rotation2d.fromRadians(m_pivotInputs.curAngle);
  }

  public boolean withinTolerance() {
    return m_pivotController.atGoal()
        && m_leftFlywheelController.atSetpoint()
        && m_rightFlywheelController.atSetpoint();
  }
}
