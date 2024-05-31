package frc.robot.subsystems.aprilTagVision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utils.GeomUtil;
import frc.robot.Constants.AprilTagVisionConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState.VisionObservation;
import frc.robot.subsystems.aprilTagVision.AprilTagVisionIO.AprilTagVisionInputs;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {
  private final AprilTagVisionIO[] m_ios;
  public final AprilTagVisionInputs[] m_inputs;

  private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  public AprilTagVision(AprilTagVisionIO... ios) {
    m_ios = ios;
    m_inputs = new AprilTagVisionInputs[ios.length];
    for (int i = 0; i < ios.length; i++) {
      m_inputs[i] = new AprilTagVisionInputs();
    }

    for (int i = 0; i < ios.length; i++) {
      lastFrameTimes.put(i, 0.0);
    }

    // create map of last detection times for each tag
    FieldConstants.kField
        .getTags()
        .forEach(
            (tag) -> {
              lastTagDetectionTimes.put(tag.ID, 0.0);
            });
  }

  @Override
  public void periodic() {
    for (int i = 0; i < m_ios.length; i++) {
      m_ios[i].updateInputs(m_inputs[i]);
      Logger.processInputs("AprilTagVision/Inst" + i, m_inputs[i]);
    }

    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses3d = new ArrayList<>();
    List<VisionObservation> allObservations = new ArrayList<>();
    int total = 0;
    for (int instanceIndex = 0; instanceIndex < m_ios.length; instanceIndex++) {
      if (RobotState.isAutonomous()) {
        total += m_inputs[instanceIndex].timestamps.length;
        if (total > 25) {
          return;
        }
      }

      for (int frameIndex = 0;
          frameIndex < m_inputs[instanceIndex].timestamps.length;
          frameIndex++) {
        lastFrameTimes.put(instanceIndex, Timer.getFPGATimestamp());

        double timestamp = m_inputs[instanceIndex].timestamps[frameIndex];
        double[] values = m_inputs[instanceIndex].frames[frameIndex];

        // exit if blank frame
        if (values.length == 0 || values[0] == 0) {
          continue;
        }

        // switch based on number of poses
        Pose3d cameraPose = null;
        Pose3d robotPose3d = null;
        boolean useVisionRotation = false;
        switch ((int) values[0]) {
          case 1:
            // one pose (multiple tags) so we use directly
            cameraPose =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));

            robotPose3d =
                cameraPose.transformBy(
                    GeomUtil.toTransform3d(AprilTagVisionConstants.kCameraPoses[instanceIndex])
                        .inverse());

            useVisionRotation = true;
            break;

          case 2:
            // two poses (one tag) so we disambiguate
            double error0 = values[1];
            double error1 = values[9];
            Pose3d cameraPose0 =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            Pose3d cameraPose1 =
                new Pose3d(
                    values[10],
                    values[11],
                    values[12],
                    new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
            Pose3d robotPose3d0 =
                cameraPose0.transformBy(
                    GeomUtil.toTransform3d(AprilTagVisionConstants.kCameraPoses[instanceIndex])
                        .inverse());
            Pose3d robotPose3d1 =
                cameraPose1.transformBy(
                    GeomUtil.toTransform3d(AprilTagVisionConstants.kCameraPoses[instanceIndex])
                        .inverse());

            // cheeck for ambiguity and select based on odometry estimated rotation
            if (error0 < error1 * AprilTagVisionConstants.kAmbiguityThreshold
                || error1 < error0 * AprilTagVisionConstants.kAmbiguityThreshold) {
              Rotation2d currentRotation =
                  frc.robot.RobotState.getInstance().getOdometryPose().getRotation();
              Rotation2d visionRotation0 = robotPose3d0.toPose2d().getRotation();
              Rotation2d visionRotation1 = robotPose3d1.toPose2d().getRotation();
              if (Math.abs(currentRotation.minus(visionRotation0).getRadians())
                  < Math.abs(currentRotation.minus(visionRotation1).getRadians())) {
                cameraPose = cameraPose0;
                robotPose3d = robotPose3d0;
              } else {
                cameraPose = cameraPose1;
                robotPose3d = robotPose3d1;
              }
              break;
            }
        }

        // if no data, exit
        if (cameraPose == null || robotPose3d == null) {
          continue;
        }

        // if robot pose is off field, exit
        if (robotPose3d.getX() < -AprilTagVisionConstants.kFieldBorderMargin
            || robotPose3d.getX()
                > FieldConstants.kFieldLength + AprilTagVisionConstants.kFieldBorderMargin
            || robotPose3d.getY() < -AprilTagVisionConstants.kFieldBorderMargin
            || robotPose3d.getY()
                > FieldConstants.kFieldWidth + AprilTagVisionConstants.kFieldBorderMargin
            || Math.abs(robotPose3d.getZ()) > AprilTagVisionConstants.kZMargin) {
          continue;
        }

        // get 2d robot pose
        Pose2d robotPose2d = robotPose3d.toPose2d();

        // get tag poses and add to list
        List<Pose3d> tagPoses = new ArrayList<>();
        int start = (values[0] == 1 ? 9 : 17); // if one pose, start at 9, else 17
        for (int i = start; i < values.length; i++) {
          int tagId = (int) values[i];
          lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
          Optional<Pose3d> tagPose = FieldConstants.kField.getTagPose((int) values[i]);
          if (tagPose.isPresent()) {
            // i'm aware that ifPresent() exists, but this is more readable
            tagPoses.add(tagPose.get());
          }
        }

        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        double avgDistance = totalDistance / tagPoses.size();

        double xyStdDev;
        if (RobotState.isAutonomous()) {
          xyStdDev =
              3.3
                  * AprilTagVisionConstants.xyStdDevCoefficient.get()
                  * Math.pow(avgDistance, 2.0)
                  / tagPoses.size();
        } else {
          xyStdDev =
              AprilTagVisionConstants.xyStdDevCoefficient.get()
                  * Math.pow(avgDistance, 2.0)
                  / tagPoses.size();
        }

        double thetaStdDev;
        if (useVisionRotation) {
          thetaStdDev =
              AprilTagVisionConstants.thetaStdDevCoefficient.get()
                  * Math.pow(avgDistance, 2.0)
                  / tagPoses.size();
        } else {
          thetaStdDev = Double.POSITIVE_INFINITY;
        }

        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/STDX", xyStdDev);
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/STDT", xyStdDev);
        allObservations.add(
            new VisionObservation(
                robotPose2d, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        allRobotPoses.add(robotPose2d);
        allRobotPoses3d.add(robotPose3d);

        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/LatencySecs",
            Timer.getFPGATimestamp() - timestamp);
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose3d", robotPose3d);
        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
      }
    }

    // log robot poses
    Logger.recordOutput("AprilTagVision/RobotPoses", allRobotPoses.toArray(Pose2d[]::new));
    Logger.recordOutput("AprilTagVision/RobotPoses3d", allRobotPoses3d.toArray(Pose3d[]::new));

    // log tag poses
    List<Pose3d> allTagPoses = new ArrayList<>();
    for (var detection : lastTagDetectionTimes.entrySet()) {
      if (Timer.getFPGATimestamp() - detection.getValue()
          < AprilTagVisionConstants.kTargetLogTimeSecs) {
        allTagPoses.add(FieldConstants.kField.getTagPose(detection.getKey()).get());
      }
    }
    Logger.recordOutput("AprilTagVision/TagPoses", allTagPoses.toArray(Pose3d[]::new));

    Logger.recordOutput("AprilTagVision/TotalObservations", allObservations.size());

    int maxObservations = 10;
    if (allObservations.size() > maxObservations) {
      allObservations = allObservations.subList(0, maxObservations);
    }

    // send to robotstate
    // yes i know this is gross but it's the best way to do it
    allObservations.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(frc.robot.RobotState.getInstance()::addVisionObservation);
  }
}
