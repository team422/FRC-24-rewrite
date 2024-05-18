package frc.lib.advantagekit;

import frc.robot.BuildConstants;
// import frc.robot.BuildConstants;
import frc.robot.Robot;
import java.io.File;
import java.nio.file.Path;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class LoggerUtil {
  public static final String kLogDirectory = "advantagekit/logs";

  public static void initializeLogger() {

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("GitDirty", getGitDirtyString());

    // Set up data receivers & replay source
    if (Robot.isSimulation()) {
      Logger.addDataReceiver(new WPILOGWriter(getLogPath()));
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
      Logger.addDataReceiver(new NT4Publisher());
    }

    // Start AdvantageKit logger
    Logger.start();
  }

  private static String getGitDirtyString() {
    switch (BuildConstants.DIRTY) {
      case 0:
        return "All changes committed";
      case 1:
        return "Uncomitted changes";
      default:
        return "Unknown";
    }
  }

  private static String getLogPath() {
    File file = Path.of(kLogDirectory).toFile();

    if (!file.exists()) {
      file.mkdirs();
    }

    return file.getAbsolutePath();
  }
}
