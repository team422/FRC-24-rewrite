package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.LoggedTunableNumber;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.pivot.IntakePivotIO;
import frc.robot.subsystems.intake.pivot.IntakePivotInputsAutoLogged;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.subsystems.intake.roller.RollerInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private RollerIO m_rollerIO;
  private IntakePivotIO m_pivotIO;

  public final RollerInputsAutoLogged m_rollerInputs;
  public final IntakePivotInputsAutoLogged m_pivotInputs;

  private ProfiledPIDController m_pivotController;

  public Intake(RollerIO io, IntakePivotIO pivotIO, ProfiledPIDController pivotController) {
    m_rollerIO = io;
    m_pivotIO = pivotIO;
    m_pivotController = pivotController;

    m_rollerInputs = new RollerInputsAutoLogged();
    m_pivotInputs = new IntakePivotInputsAutoLogged();
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_pivotController.setP(IntakeConstants.kPivotP.get());
          m_pivotController.setI(IntakeConstants.kPivotI.get());
          m_pivotController.setD(IntakeConstants.kPivotD.get());
        },
        IntakeConstants.kPivotP,
        IntakeConstants.kPivotI,
        IntakeConstants.kPivotD);

    m_pivotIO.updateInputs(m_pivotInputs);
    m_rollerIO.updateInputs(m_rollerInputs);

    double pivotPidVoltage = m_pivotController.calculate(m_pivotInputs.curAngle);
    m_pivotIO.setVoltage(pivotPidVoltage);

    Logger.processInputs("Intake/Pivot", m_pivotInputs);
    Logger.processInputs("Intake/Roller", m_rollerInputs);

    Logger.recordOutput("Intake/Pivot/PIDVoltage", pivotPidVoltage);
    Logger.recordOutput(
        "Intake/Pivot/DesiredAngle",
        Units.radiansToDegrees(m_pivotController.getSetpoint().position));
    Logger.recordOutput(
        "Intake/Pivot/CurrentAngle", Units.radiansToDegrees(m_pivotInputs.curAngle));
  }

  public void setRollerVoltage(double voltage) {
    m_rollerIO.setVoltage(voltage);
  }

  public void setPivotAngle(Rotation2d angle) {
    m_pivotController.setGoal(angle.getRadians());
  }

  public Command runRollerCommand(double voltage) {
    return Commands.runOnce(() -> m_rollerIO.setVoltage(voltage));
  }

  public Command setPivotAngleCommand(Rotation2d angle) {
    return Commands.runOnce(() -> m_pivotController.setGoal(angle.getRadians()));
  }

  public Rotation2d getPivotAngle() {
    return Rotation2d.fromRadians(m_pivotInputs.curAngle);
  }
}
