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

import edu.wpi.first.math.controller.ProfiledPIDController;
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
  public static final boolean tuningMode = false;

  public static final class ShooterConstants {

    // Shooter pivot
    public static final double kPivotGearRatio = 46.722;
    // TODO: untuned values, fix later
    public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(55);
    public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(15);
    public static final Rotation2d kHomeAngle = Rotation2d.fromDegrees(33);

    public static final Rotation2d kPivotOffset = Rotation2d.fromDegrees(170);

    public static final LoggedTunableNumber kPivotP =
        new LoggedTunableNumber("Shooter Pivot P", 1.0);
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
        new LoggedTunableNumber("Shooter Flywheel P", 0.7);
    public static final LoggedTunableNumber kLeftFlywheelI =
        new LoggedTunableNumber("Shooter Flywheel I", 0.0);
    public static final LoggedTunableNumber kLeftFlywheelD =
        new LoggedTunableNumber("Shooter Flywheel D", 0.0);
    public static final double kLeftFlywheelVelocity = 10.0;
    public static final double kLeftFlywheelAcceleration = 15.0;
    public static final ProfiledPIDController kLeftFlywheelController =
        new ProfiledPIDController(
            kLeftFlywheelP.get(),
            kLeftFlywheelI.get(),
            kLeftFlywheelD.get(),
            new Constraints(kLeftFlywheelVelocity, kLeftFlywheelAcceleration));

    public static final LoggedTunableNumber kRightFlywheelP =
        new LoggedTunableNumber("Shooter Flywheel P", 0.7);
    public static final LoggedTunableNumber kRightFlywheelI =
        new LoggedTunableNumber("Shooter Flywheel I", 0.0);
    public static final LoggedTunableNumber kRightFlywheelD =
        new LoggedTunableNumber("Shooter Flywheel D", 0.0);
    public static final double kRightFlywheelVelocity = 10.0;
    public static final double kRightFlywheelAcceleration = 15.0;
    public static final ProfiledPIDController kRightFlywheelController =
        new ProfiledPIDController(
            kRightFlywheelP.get(),
            kRightFlywheelI.get(),
            kRightFlywheelD.get(),
            new Constraints(kRightFlywheelVelocity, kRightFlywheelAcceleration));
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
    public static final int kIntakePivot = 0;

    public static final int kClimbUp = 26;
    public static final int kClimbDown = 25;

    public static final int kAmp = 54;
  }
}
