package frc.robot.subsystems.aprilTagVision;

import frc.lib.advantagekit.LoggedIO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AprilTagVisionIO extends LoggedIO<AprilTagVisionIO.AprilTagVisionInputs> {
  public static class AprilTagVisionInputs implements LoggableInputs {
    public double[] timestamps = new double[0];
    public double[][] frames = new double[0][0];
    public double[] demoFrame = new double[0];
    public long fps = 0;

    @Override
    public void toLog(LogTable table) {
      table.put("Timestamps", timestamps);
      table.put("FrameCount", timestamps.length);
      for (int i = 0; i < frames.length; i++) {
        table.put("Frame/" + i, frames[i]);
      }
      table.put("DemoFrame", demoFrame);
      table.put("FPS", fps);
    }

    @Override
    public void fromLog(LogTable table) {
      timestamps = table.get("Timestamps", new double[] {0.0});
      int frameCount = table.get("FrameCount", 0);
      frames = new double[frameCount][];
      for (int i = 0; i < frameCount; i++) {
        frames[i] = table.get("Frame/" + i, new double[0]);
      }
      demoFrame = table.get("DemoFrame", new double[0]);
      fps = table.get("FPS", 0);
    }
  }
}
