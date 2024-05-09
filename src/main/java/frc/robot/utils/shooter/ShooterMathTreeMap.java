package frc.robot.utils.shooter;

import frc.robot.subsystems.shooter.Shooter.ShooterPosition;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterMathTreeMap implements ShooterMath {
    private InterpolatingDoubleTreeMap m_shootSpeedLeft;
    private InterpolatingDoubleTreeMap m_shootSpeedRight;
    // in degrees
    private InterpolatingDoubleTreeMap m_shootAngle;

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
        return new ShooterPosition(Rotation2d.fromDegrees(m_shootAngle.get(distance)), m_shootSpeedLeft.get(distance),
                m_shootSpeedRight.get(distance));
    }

    @Override
    public double calculateLeftFlywheelVelocity(double distance) {
        return m_shootSpeedLeft.get(distance);
    }

    @Override
    public double calculateRightFlywheelVelocity(double distance) {
        return m_shootSpeedRight.get(distance);
    }

    @Override
    public double calculatePivotAngle(double distance, double angle) {
        return m_shootAngle.get(distance);
    }
}
