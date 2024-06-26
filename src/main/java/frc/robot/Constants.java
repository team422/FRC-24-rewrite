// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.lib.utils.LoggedTunableNumber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean tuningMode = true;

  public static final class ShooterConstants {

    // Shooter pivot
    public static final double kPivotGearRatio = 46.722;

    public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(70);
    public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(15);
    public static final Rotation2d kHomeAngle = Rotation2d.fromDegrees(33);

    public static final Rotation2d kPivotOffset = Rotation2d.fromDegrees(170);

    public static final LoggedTunableNumber kPivotP =
        new LoggedTunableNumber("Shooter Pivot P", 4.0);
    public static final LoggedTunableNumber kPivotI =
        new LoggedTunableNumber("Shooter Pivot I", 0.0);
    public static final LoggedTunableNumber kPivotD =
        new LoggedTunableNumber("Shooter Pivot D", 0.0);
    public static final double kPivotVelocity = 5.0;
    public static final double kPivotAcceleration = 10.0;
    public static final ProfiledPIDController kPivotController =
        new ProfiledPIDController(
            kPivotP.get(),
            kPivotI.get(),
            kPivotD.get(),
            new Constraints(kPivotVelocity, kPivotAcceleration));

    // Shooter flywheel
    public static final double kFlywheelDiameter = Units.inchesToMeters(4.0);

    public static final LoggedTunableNumber kLeftFlywheelP =
        new LoggedTunableNumber("Shooter Left Flywheel P", 1.0);
    public static final LoggedTunableNumber kLeftFlywheelI =
        new LoggedTunableNumber("Shooter Left Flywheel I", 0.0);
    public static final LoggedTunableNumber kLeftFlywheelD =
        new LoggedTunableNumber("Shooter Left Flywheel D", 0.0);
    public static final PIDController kLeftFlywheelController =
        new PIDController(kLeftFlywheelP.get(), kLeftFlywheelI.get(), kLeftFlywheelD.get());

    public static final LoggedTunableNumber kRightFlywheelP =
        new LoggedTunableNumber("Shooter Right Flywheel P", 1.0);
    public static final LoggedTunableNumber kRightFlywheelI =
        new LoggedTunableNumber("Shooter Right Flywheel I", 0.0);
    public static final LoggedTunableNumber kRightFlywheelD =
        new LoggedTunableNumber("Shooter Right Flywheel D", 0.0);
    public static final PIDController kRightFlywheelController =
        new PIDController(kRightFlywheelP.get(), kRightFlywheelI.get(), kRightFlywheelD.get());

    public static final LoggedTunableNumber kRightFlywheelKs =
        new LoggedTunableNumber("Shooter Right Flywheel kS", 0.38);
    public static final LoggedTunableNumber kRightFlywheelKv =
        new LoggedTunableNumber("Shooter Right Flywheel kV", 0.27);
    public static final SimpleMotorFeedforward kRightFlywheelFeedforward =
        new SimpleMotorFeedforward(kRightFlywheelKs.get(), kRightFlywheelKv.get());

    public static final LoggedTunableNumber kLeftFlywheelKs =
        new LoggedTunableNumber("Shooter Left Flywheel kS", 0.32);
    public static final LoggedTunableNumber kLeftFlywheelKv =
        new LoggedTunableNumber("Shooter Left Flywheel kV", 0.27);
    public static final SimpleMotorFeedforward kLeftFlywheelFeedforward =
        new SimpleMotorFeedforward(kLeftFlywheelKs.get(), kLeftFlywheelKv.get());
  }

  public static final class IntakeConstants {
    public static final double kPivotGearRatio = 36.0 / 16;

    public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(124);
    public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(5);
    public static final Rotation2d kHomeAngle = Rotation2d.fromDegrees(120);
    public static final Rotation2d kDeployedAngle = Rotation2d.fromDegrees(15);

    // don't ask me why it's negative, just a hardware thing
    public static final double kDeployRollerVoltage = -6.0;

    public static final Rotation2d kPivotOffset = Rotation2d.fromDegrees(123);

    public static final LoggedTunableNumber kPivotP =
        new LoggedTunableNumber("Intake Pivot P", 3.0);
    public static final LoggedTunableNumber kPivotI =
        new LoggedTunableNumber("Intake Pivot I", 0.0);
    public static final LoggedTunableNumber kPivotD =
        new LoggedTunableNumber("Intake Pivot D", 0.0);
    public static final double kPivotVelocity = 25.0;
    public static final double kPivotAcceleration = 30.0;
    public static final ProfiledPIDController kPivotController =
        new ProfiledPIDController(
            kPivotP.get(),
            kPivotI.get(),
            kPivotD.get(),
            new Constraints(kPivotVelocity, kPivotAcceleration));
  }

  public static class IndexerConstants {
    public static final double kFeederVoltage = 6.0;
    public static final double kKickerVoltage = 6.0;
  }

  public static final class Ports {
    public static final int kShooterPivotLeader = 39;
    public static final int kShooterPivotFollower = 40;
    public static final int kShooterPivotEncoder = 9;

    public static final int kFlywheelLeft = 35;
    public static final int kFlywheelRight = 36;

    public static final int kKicker = 31;
    public static final int kFeeder = 30;
    public static final int kIndexerBeamBreakOne = 2;
    public static final int kIndexerBeamBreakTwo = 3;

    public static final int kIntakeRoller = 43;
    public static final int kIntakePivot = 33;

    public static final int kClimbUp = 26;
    public static final int kClimbDown = 25;

    public static final int kAmp = 54;

    public static final int kLed = 9;
  }
}
