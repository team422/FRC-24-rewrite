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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.lib.utils.AllianceFlipUtil;
import frc.lib.utils.LoggedTunableNumber;
import java.util.HashMap;

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

    // setpoints for ImmerseCon (firing into crowd)
    public static final LoggedTunableNumber kLeftFlywheelLowVelocity =
        new LoggedTunableNumber("Shooter Left Flywheel Low Velocity", 9.0);
    public static final LoggedTunableNumber kRightFlywheelLowVelocity =
        new LoggedTunableNumber("Shooter Right Flywheel Low Velocity", 11.0);
    public static final LoggedTunableNumber kPivotLowAngle =
        new LoggedTunableNumber("Shooter Pivot Low Angle", 20);

    public static final LoggedTunableNumber kLeftFlywheelMidVelocity =
        new LoggedTunableNumber("Shooter Left Flywheel Mid Velocity", 9.0);
    public static final LoggedTunableNumber kRightFlywheelMidVelocity =
        new LoggedTunableNumber("Shooter Right Flywheel Mid Velocity", 11.0);
    public static final LoggedTunableNumber kPivotMidAngle =
        new LoggedTunableNumber("Shooter Pivot Mid Angle", 35);

    public static final LoggedTunableNumber kLeftFlywheelHighVelocity =
        new LoggedTunableNumber("Shooter Left Flywheel High Velocity", 9.0);
    public static final LoggedTunableNumber kRightFlywheelHighVelocity =
        new LoggedTunableNumber("Shooter Right Flywheel High Velocity", 11.0);
    public static final LoggedTunableNumber kPivotHighAngle =
        new LoggedTunableNumber("Shooter Pivot High Angle", 55);
  }

  public static final class IntakeConstants {
    public static final double kPivotGearRatio = 36.0 / 16;

    public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(125);
    public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(5);
    public static final Rotation2d kHomeAngle = Rotation2d.fromDegrees(124);
    public static final Rotation2d kDeployedAngle = Rotation2d.fromDegrees(15);

    // don't ask me why it's negative, just a hardware thing
    public static final double kDeployRollerVoltage = -6.0;

    public static final Rotation2d kPivotOffset = Rotation2d.fromDegrees(124);

    public static final LoggedTunableNumber kPivotP =
        new LoggedTunableNumber("Intake Pivot P", 3.0);
    public static final LoggedTunableNumber kPivotI =
        new LoggedTunableNumber("Intake Pivot I", 0.0);
    public static final LoggedTunableNumber kPivotD =
        new LoggedTunableNumber("Intake Pivot D", 0.04);
    public static final PIDController kPivotController =
        new PIDController(kPivotP.get(), kPivotI.get(), kPivotD.get());
  }

  public static class IndexerConstants {
    public static final double kFeederVoltage = 6.0;
    public static final double kKickerVoltage = 6.0;
  }

  public static final class FieldConstants {
    public static final class Source {
      public static final Translation2d kSource = new Translation2d(15.696, 0.701);
    }

    public static final Translation3d kTopRightSpeaker =
        new Translation3d(
            Units.inchesToMeters(0.0), Units.inchesToMeters(238.815), Units.inchesToMeters(83.091));

    public static final Translation3d kTopLeftSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(197.765),
            Units.inchesToMeters(83.091));

    public static final Translation3d kBottomRightSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
    public static final Translation3d kBottomLeftSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

    /** Center of the speaker opening (blue alliance) */
    public static final Translation3d kCenterSpeakerOpening =
        kBottomLeftSpeaker.interpolate(kTopRightSpeaker, 0.5);

    public static final double kFieldLength = Units.inchesToMeters(651.223);
    public static final double kFieldWidth = Units.inchesToMeters(323.277);
    public static final double kWingX = Units.inchesToMeters(229.201);
    public static final double kPodiumX = Units.inchesToMeters(126.75);
    public static final double kStartingLineX = Units.inchesToMeters(74.111);

    public static final double kAprilTagWidth = Units.inchesToMeters(6.5);
    public static final AprilTagFieldLayout kField =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final Pose2d kAmpBlue = new Pose2d(1.749, 7.82, Rotation2d.fromDegrees(90));
    public static final Pose2d kDailedShot = new Pose2d(2.95, 4.08, new Rotation2d(145.00));
    public static final Translation2d kCorner = new Translation2d(0, 7.82);
    public static final Translation2d kFeederAim = new Translation2d(1, 6.82);
    public static final Translation2d kSourceMidShot = new Translation2d(8.04, 2);

    public static final class StagingLocations {
      public static final double kCenterlineX = kFieldLength / 2.0;

      // need to update
      public static final double kCenterlineFirstY = Units.inchesToMeters(29.638);
      public static final double kCenterlineSeparationY = Units.inchesToMeters(66);
      public static final double kSpikeX = Units.inchesToMeters(114);
      // need
      public static final double kSpikeFirstY = Units.inchesToMeters(161.638);
      public static final double kSpikeSeparationY = Units.inchesToMeters(57);

      public static final Translation2d[] kCenterlineTranslations = new Translation2d[5];
      public static final Translation2d[] kSpikeTranslations = new Translation2d[3];

      static {
        for (int i = 0; i < kCenterlineTranslations.length; i++) {
          kCenterlineTranslations[i] =
              new Translation2d(kCenterlineX, kCenterlineFirstY + (i * kCenterlineSeparationY));
        }
      }

      static {
        for (int i = 0; i < kSpikeTranslations.length; i++) {
          kSpikeTranslations[i] =
              new Translation2d(kSpikeX, kSpikeFirstY + (i * kSpikeSeparationY));
        }
      }
    }

    public static final HashMap<String, Pose2d> getGamePieces() {
      HashMap<String, Pose2d> gamePieces = new HashMap<String, Pose2d>();
      // gamePieces.put("CloseRight", new Pose2d(Units.inchesToMeters(325),
      // Units.inchesToMeters(160), new Rotation2d()));
      for (int i = FieldConstants.StagingLocations.kSpikeTranslations.length - 1; i >= 0; i--) {
        gamePieces.put(
            i + "Spike",
            new Pose2d(
                AllianceFlipUtil.apply(FieldConstants.StagingLocations.kSpikeTranslations[i]),
                new Rotation2d()));
      }
      for (int i = FieldConstants.StagingLocations.kCenterlineTranslations.length - 1;
          i >= 0;
          i--) {
        gamePieces.put(
            i + "Centerline",
            new Pose2d(
                AllianceFlipUtil.apply(FieldConstants.StagingLocations.kCenterlineTranslations[i]),
                new Rotation2d()));
      }

      return gamePieces;
    }
  }

  public static final class AprilTagVisionConstants {
    public static final double kAmbiguityThreshold = 0.4;
    public static final double kTargetLogTimeSecs = 0.1;
    public static final double kFieldBorderMargin = 0.5;
    public static final double kZMargin = 0.75;
    public static final LoggedTunableNumber xyStdDevCoefficient =
        new LoggedTunableNumber("xyStdDevCoefficient", 0.005, "Cameras");
    public static final LoggedTunableNumber thetaStdDevCoefficient =
        new LoggedTunableNumber("thetaStdDevCoefficient", 0.01, "Cameras");
    ;

    public static final Pose3d[] kCameraPoses =
        new Pose3d[] {
          new Pose3d( // left intake side
              Units.inchesToMeters(-6.096),
              Units.inchesToMeters(-7.884),
              Units.inchesToMeters(25.413),
              new Rotation3d(0.0, Units.degreesToRadians(-10), 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(180 + 20.0)))),
          new Pose3d(
              Units.inchesToMeters(-6.096),
              Units.inchesToMeters(7.884),
              Units.inchesToMeters(25.413),
              new Rotation3d(0.0, Units.degreesToRadians(-10), 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(180 - 20.0)))),
          new Pose3d(
              Units.inchesToMeters(10.203),
              Units.inchesToMeters(-5.5422),
              Units.inchesToMeters(7.41288314),
              new Rotation3d(0.0, Units.degreesToRadians(-35), 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-10)))),
          new Pose3d(
              Units.inchesToMeters(10.203),
              Units.inchesToMeters(5.5422),
              Units.inchesToMeters(7.41288314),
              new Rotation3d(0.0, Units.degreesToRadians(-35), 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(10)))),
        };

    public static final String[] kInstanceNames = {
      "northstar_0", "northstar_1", "northsstar_2", "northstar_3"
    };

    public static final String[] kCameraIds =
        new String[] {
          "/dev/video0", "/dev/v4l/by-path/platform-fc880000.usb-usb-0:1:1.0-video-index0"
        };
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
