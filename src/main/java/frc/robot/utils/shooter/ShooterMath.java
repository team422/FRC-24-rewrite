package frc.robot.utils.shooter;

import frc.robot.subsystems.shooter.Shooter.ShooterPosition;

public interface ShooterMath {
    public ShooterPosition calculateShooterPosition(double distance, double angle);

    public double calculateLeftFlywheelVelocity(double distance);

    public double calculateRightFlywheelVelocity(double distance);

    public double calculatePivotAngle(double distance, double angle);
}
