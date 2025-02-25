package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.hal.HALUtil;
import java.util.HashMap;
import java.util.Map;

public class Stopwatch {
  public static void start(String name) {
    lastTimestamps.put(name, getTimestamp());
  }

  public static void stop(String name) {
    double timestamp = getTimestamp();
    double lastTimestamp = lastTimestamps.get(name);
    DogLog.log(name, timestamp - lastTimestamp);
  }

  public static void skip(String name) {
    DogLog.log(name, -1.0);
  }

  private static final Map<String, Double> lastTimestamps = new HashMap<>();

  private static double getTimestamp() {
    return HALUtil.getFPGATime() / 1e3;
  }

  private Stopwatch() {}
}
