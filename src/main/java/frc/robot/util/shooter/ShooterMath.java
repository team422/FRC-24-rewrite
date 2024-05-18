package frc.robot.util.shooter;

import frc.robot.subsystems.shooter.Shooter.ShooterPosition;

public interface ShooterMath {
  public ShooterPosition calculateShooterPosition(double distance);

  public double calculateLeftFlywheelVelocity(double distance);

  public double calculateRightFlywheelVelocity(double distance);

  public double calculatePivotAngle(double distance);
}
