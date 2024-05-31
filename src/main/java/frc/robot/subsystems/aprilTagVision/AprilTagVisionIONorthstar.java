package frc.robot.subsystems.aprilTagVision;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.utils.Alert;
import frc.robot.Constants.FieldConstants;

public class AprilTagVisionIONorthstar implements AprilTagVisionIO {
  private static final int kCameraResolutionWidth = 1600;
  private static final int kCameraResolutionHeight = 1200;
  private static final int kCameraAutoExposure = 1;
  private static final int kCameraExposure = 150;
  private static final int kCameraGain = 25;

  private final DoubleArraySubscriber observationSubscriber;
  private final DoubleArraySubscriber demoObservationSubscriber;
  private final IntegerSubscriber fpsSubscriber;

  private static final double disconnectedTimeout = 0.5;
  private final Timer disconnectedTimer = new Timer();
  private final Alert disconnectedAlert;

  public AprilTagVisionIONorthstar(String instanceId, String cameraId) {
    var northstarTable = NetworkTableInstance.getDefault().getTable(instanceId);

    var configTable = northstarTable.getSubTable("config");
    configTable.getStringTopic("camera_id").publish().set(cameraId);
    configTable.getIntegerTopic("camera_resolution_width").publish().set(kCameraResolutionWidth);
    configTable.getIntegerTopic("camera_resolution_height").publish().set(kCameraResolutionHeight);
    configTable.getIntegerTopic("camera_auto_exposure").publish().set(kCameraAutoExposure);
    configTable.getIntegerTopic("camera_exposure").publish().set(kCameraExposure);
    configTable.getIntegerTopic("camera_gain").publish().set(kCameraGain);
    configTable.getDoubleTopic("fiducial_size_m").publish().set(FieldConstants.kAprilTagWidth);

    try {
      configTable
          .getStringTopic("tag_layout")
          .publish()
          .set(new ObjectMapper().writeValueAsString(FieldConstants.kField));
    } catch (JsonProcessingException e) {
      throw new RuntimeException("Failed to serializee AprilTag layout JSON for Northstar", e);
    }

    var outputTable = northstarTable.getSubTable("output");
    observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[0], PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    demoObservationSubscriber =
        outputTable
            .getDoubleArrayTopic("demo_observations")
            .subscribe(
                new double[0], PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    fpsSubscriber =
        outputTable
            .getIntegerTopic("fps")
            .subscribe(0, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

    disconnectedAlert = new Alert("No data from Northstar " + instanceId, Alert.AlertType.ERROR);
    disconnectedTimer.start();
  }

  @Override
  public void updateInputs(AprilTagVisionInputs inputs) {
    var queue = observationSubscriber.readQueue();
    inputs.timestamps = new double[queue.length];
    inputs.frames = new double[queue.length][];
    for (int i = 0; i < queue.length; i++) {
      inputs.timestamps[i] = queue[i].timestamp / 1e6; // convert from microseconds to seconds
      inputs.frames[i] = queue[i].value;
    }
    inputs.demoFrame = new double[0];
    for (double[] frame : demoObservationSubscriber.readQueueValues()) {
      inputs.demoFrame = frame;
    }
    inputs.fps = fpsSubscriber.get();

    // update alert
    if (queue.length > 0) {
      disconnectedTimer.reset();
    }
    disconnectedAlert.set(disconnectedTimer.hasElapsed(disconnectedTimeout));
  }
}
