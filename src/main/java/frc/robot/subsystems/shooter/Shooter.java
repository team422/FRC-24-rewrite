package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.LoggedTunableNumber;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelInputsAutoLogged;
import frc.robot.subsystems.shooter.pivot.ShooterPivotIO;
import frc.robot.subsystems.shooter.pivot.ShooterPivotInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /**
   * Represents the desired state of the shooter, including flywheel and pivot measurements
   */
  public static class ShooterPosition {
    public Rotation2d pivotAngle;
    public double leftFlywheelVelocity;
    public double rightFlywheelVelocity;

    public ShooterPosition(Rotation2d pivotAngle, double leftFlywheelVelocity, double rightFlywheelVelocity) {
      this.pivotAngle = pivotAngle;
      this.leftFlywheelVelocity = leftFlywheelVelocity;
      this.rightFlywheelVelocity = rightFlywheelVelocity;
    }
  }

  private ShooterPivotIO m_pivotIO;
  private FlywheelIO m_flywheelIO;

  public final ShooterPivotInputsAutoLogged m_pivotInputs;
  public final FlywheelInputsAutoLogged m_flywheelInputs;

  private ProfiledPIDController m_pivotController;
  private ProfiledPIDController m_leftFlywheelController;
  private ProfiledPIDController m_rightFlywheelController;

  public Shooter(
      ShooterPivotIO pivotIO,
      FlywheelIO flywheelIO,
      ProfiledPIDController pivotController,
      ProfiledPIDController flywheelController) {

    m_pivotIO = pivotIO;
    m_flywheelIO = flywheelIO;

    m_pivotController = pivotController;
    m_leftFlywheelController = flywheelController;
    m_rightFlywheelController = flywheelController;

    m_pivotInputs = new ShooterPivotInputsAutoLogged();
    m_flywheelInputs = new FlywheelInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // update pid if changed
    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      m_pivotController.setP(ShooterConstants.kPivotP.get());
      m_pivotController.setI(ShooterConstants.kPivotI.get());
      m_pivotController.setD(ShooterConstants.kPivotD.get());
    }, ShooterConstants.kPivotP, ShooterConstants.kPivotI, ShooterConstants.kPivotD);

    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      m_leftFlywheelController.setP(ShooterConstants.kFlywheelP.get());
      m_leftFlywheelController.setI(ShooterConstants.kFlywheelI.get());
      m_leftFlywheelController.setD(ShooterConstants.kFlywheelD.get());
      m_rightFlywheelController.setP(ShooterConstants.kFlywheelP.get());
      m_rightFlywheelController.setI(ShooterConstants.kFlywheelI.get());
      m_rightFlywheelController.setD(ShooterConstants.kFlywheelD.get());
    }, ShooterConstants.kFlywheelP, ShooterConstants.kFlywheelI, ShooterConstants.kFlywheelD);

    m_pivotIO.updateInputs(m_pivotInputs);
    m_flywheelIO.updateInputs(m_flywheelInputs);

    double pivotPidVoltage = m_pivotController.calculate(m_pivotInputs.curAngle);
    m_pivotIO.setVoltage(pivotPidVoltage);

    double leftFlywheelPidVoltage =
        m_leftFlywheelController.calculate(m_flywheelInputs.linearVelocity[0]);
    double rightFlywheelPidVoltage =
        m_rightFlywheelController.calculate(m_flywheelInputs.linearVelocity[1]);
    m_flywheelIO.setVoltage(leftFlywheelPidVoltage, rightFlywheelPidVoltage);

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
        "Shooter/Flywheel/LeftFlywheelVelocity", m_flywheelInputs.linearVelocity[0]);
    Logger.recordOutput(
        "Shooter/Flywheel/RightFlywheelVelocity", m_flywheelInputs.linearVelocity[1]);
    Logger.recordOutput(
        "Shooter/Flywheel/LeftFlywheelDesiredVelocity",
        m_leftFlywheelController.getGoal().position);
    Logger.recordOutput(
        "Shooter/Flywheel/RightFlywheelDesiredVelocity",
        m_rightFlywheelController.getGoal().position);
  }

  public void setPivotAngle(Rotation2d angle) {
    m_pivotController.setGoal(angle.getRadians());
  }

  /** Set the flywheel velocity in meters per second */
  public void setFlywheelVelocity(double leftVelocity, double rightVelocity) {
    m_leftFlywheelController.setGoal(leftVelocity);
    m_rightFlywheelController.setGoal(rightVelocity);
  }

  public Command setPivotAngleCommand(Rotation2d angle) {
    return Commands.runOnce(() -> setPivotAngle(angle));
  }

  /** Set the flywheel velocity in meters per second */
  public Command setFlywheelVelocityCommand(double leftRPM, double rightRPM) {
    return Commands.runOnce(() -> setFlywheelVelocity(leftRPM, rightRPM));
  }

  public void setShooter(ShooterPosition state) {
    setPivotAngle(state.pivotAngle);
    setFlywheelVelocity(state.leftFlywheelVelocity, state.rightFlywheelVelocity);
  }

  public Command setShooterCommand(ShooterPosition state) {
    return Commands.runOnce(() -> setShooter(state));
  }

  public Rotation2d getPivotAngle() {
    return Rotation2d.fromRadians(m_pivotInputs.curAngle);
  }
}
