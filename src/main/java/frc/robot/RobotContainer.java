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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsPS5;
import frc.robot.oi.DriverControlsXbox;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOFalcon;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.pivot.IntakePivotIONeo;
import frc.robot.subsystems.intake.pivot.IntakePivotIOSim;
import frc.robot.subsystems.intake.roller.RollerIOKraken;
import frc.robot.subsystems.intake.roller.RollerIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOKraken;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.pivot.ShooterPivotIOFalcon;
import frc.robot.subsystems.shooter.pivot.ShooterPivotIOSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private RobotState m_robotState;

  // Subsystems
  private Drive m_drive;
  private Shooter m_shooter;
  private Indexer m_indexer;
  private Intake m_intake;

  // Controllers
  private DriverControls m_driverControls;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureSubsystems();
    configureControllers();
    configureButtonBindings();
  }

  private void configureSubsystems() {
    if (RobotBase.isReal()) {
      m_drive =
          new Drive(
              new GyroIOPigeon2(true),
              new ModuleIOTalonFX(0),
              new ModuleIOTalonFX(1),
              new ModuleIOTalonFX(2),
              new ModuleIOTalonFX(3));
      m_shooter =
          new Shooter(
              new ShooterPivotIOFalcon(
                  Ports.kShooterPivotLeader,
                  Ports.kShooterPivotFollower,
                  Ports.kShooterPivotEncoder),
              new FlywheelIOKraken(Ports.kFlywheelLeft, Ports.kFlywheelRight),
              ShooterConstants.kPivotController,
              ShooterConstants.kLeftFlywheelController,
              ShooterConstants.kRightFlywheelController,
              ShooterConstants.kLeftFlywheelFeedforward,
              ShooterConstants.kRightFlywheelFeedforward);
      m_indexer =
          new Indexer(
              new IndexerIOFalcon(
                  Ports.kKicker,
                  Ports.kFeeder,
                  Ports.kIndexerBeamBreakOne,
                  Ports.kIndexerBeamBreakTwo));

      m_intake =
          new Intake(
              new RollerIOKraken(Ports.kIntakeRoller),
              new IntakePivotIONeo(Ports.kIntakePivot),
              IntakeConstants.kPivotController);
    } else {
      m_drive =
          new Drive(
              new GyroIOPigeon2(true),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim(),
              new ModuleIOSim());
      m_shooter =
          new Shooter(
              new ShooterPivotIOSim(),
              new FlywheelIOSim(),
              ShooterConstants.kPivotController,
              ShooterConstants.kLeftFlywheelController,
              ShooterConstants.kRightFlywheelController,
              ShooterConstants.kLeftFlywheelFeedforward,
              ShooterConstants.kRightFlywheelFeedforward);
      m_indexer = new Indexer(new IndexerIOSim());

      m_intake =
          new Intake(new RollerIOSim(), new IntakePivotIOSim(), IntakeConstants.kPivotController);
    }

    m_shooter.setPivotAngle(ShooterConstants.kHomeAngle);
    m_intake.setPivotAngle(IntakeConstants.kHomeAngle);

    m_robotState = RobotState.startInstance(m_drive, m_shooter, m_indexer, m_intake);
  }

  private void configureControllers() {
    // usually use ps5 on actual robot but xbox is better for keyboard testing
    if (RobotBase.isReal()) {
      m_driverControls = new DriverControlsPS5(1);
    } else {
      m_driverControls = new DriverControlsXbox(1);
    }
  }

  private void configureButtonBindings() {
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            m_driverControls::getDriveForward,
            m_driverControls::getDriveLeft,
            m_driverControls::getDriveRotation));

    m_driverControls
        .testShooter()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_shooter.setFlywheelVelocity(
                      ShooterConstants.kTestLeftFlywheelSpeed.get(),
                      ShooterConstants.kTestRightFlywheelSpeed.get());
                  m_shooter.setPivotAngle(Rotation2d.fromDegrees(50));
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_shooter.setFlywheelVelocity(0.0, 0.0);
                  m_shooter.setPivotAngle(ShooterConstants.kHomeAngle);
                }));

    m_driverControls
        .testKicker()
        .onTrue(m_indexer.runKicker(6.0))
        .onFalse(m_indexer.runKicker(0.0));

    m_driverControls
        .testFeeder()
        .onTrue(m_indexer.runFeeder(6.0))
        .onFalse(m_indexer.runFeeder(0.0));


    m_driverControls
        .testIntake()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_intake.setRollerVoltage(8);
                  m_intake.setPivotAngle(Rotation2d.fromDegrees(IntakeConstants.kTestPivotAngle.get()));
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_intake.setRollerVoltage(0);
                  m_intake.setPivotAngle(IntakeConstants.kHomeAngle);
                }));
  }

  public void updateRobotState() {
    m_robotState.updateRobotState();
  }
}
