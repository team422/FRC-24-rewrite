package frc.robot.subsystems.amp;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.LoggedTunableNumber;
import frc.robot.Constants.AmpConstants;
import org.littletonrobotics.junction.Logger;

public class Amp extends SubsystemBase {
  public AmpIO m_io;
  public AmpInputsAutoLogged m_inputs;
  private ProfiledPIDController m_controller;

  public Amp(AmpIO io, ProfiledPIDController controller) {
    m_io = io;
    m_inputs = new AmpInputsAutoLogged();
    m_controller = controller;
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          m_controller.setP(AmpConstants.kP.get());
          m_controller.setI(AmpConstants.kI.get());
          m_controller.setD(AmpConstants.kD.get());
        },
        AmpConstants.kP,
        AmpConstants.kI,
        AmpConstants.kD);

    m_io.updateInputs(m_inputs);

    double pidVoltage = m_controller.calculate(m_inputs.curAngle);
    m_io.setVoltage(pidVoltage);

    Logger.processInputs("Amp", m_inputs);

    Logger.recordOutput("Amp/PIDVoltage", pidVoltage);
    Logger.recordOutput("Amp/CurAngle", Units.rotationsToDegrees(m_inputs.curAngle));
    Logger.recordOutput(
        "Amp/DesiredAngle", Units.rotationsToDegrees(m_controller.getGoal().position));
  }

  public void setAmpAngle(Rotation2d angle) {
    m_controller.setGoal(angle.getRotations());
  }
}
