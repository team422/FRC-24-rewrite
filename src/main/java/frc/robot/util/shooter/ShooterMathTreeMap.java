package frc.robot.util.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.shooter.Shooter.ShooterPosition;

public class ShooterMathTreeMap implements ShooterMath {
  private InterpolatingDoubleTreeMap m_shootSpeedLeft;
  private InterpolatingDoubleTreeMap m_shootSpeedRight;
  // in degrees
  private InterpolatingDoubleTreeMap m_shootAngle;

  private final double kMeasurementOffset = 0.0;
  // when calibrating, this is the x distance from 0.0 on the tape measure to the speaker hole

  public ShooterMathTreeMap() {
    m_shootSpeedLeft = new InterpolatingDoubleTreeMap();
    m_shootSpeedRight = new InterpolatingDoubleTreeMap();
    m_shootAngle = new InterpolatingDoubleTreeMap();

    // Left data

    // Right data

    // Angle data

  }

  @Override
  public ShooterPosition calculateShooterPosition(double distance, double angle) {
    distance += kMeasurementOffset;
    return new ShooterPosition(
        Rotation2d.fromDegrees(m_shootAngle.get(distance)),
        m_shootSpeedLeft.get(distance),
        m_shootSpeedRight.get(distance));
  }

  @Override
  public double calculateLeftFlywheelVelocity(double distance) {
    distance += kMeasurementOffset;
    return m_shootSpeedLeft.get(distance);
  }

  @Override
  public double calculateRightFlywheelVelocity(double distance) {
    distance += kMeasurementOffset;
    return m_shootSpeedRight.get(distance);
  }

  @Override
  public double calculatePivotAngle(double distance, double angle) {
    distance += kMeasurementOffset;
    return m_shootAngle.get(distance);
  }
}
