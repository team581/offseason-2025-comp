package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.config.RobotConfig;
import java.util.function.BooleanSupplier;

public class FeatureFlag {
  public static BooleanSupplier of(String name, boolean defaultValue) {
    if (RobotConfig.IS_DEVELOPMENT) {
      var topic = NetworkTableInstance.getDefault().getBooleanTopic("Robot/FeatureFlags/" + name);

      topic.publish().set(defaultValue);

      return topic.subscribe(defaultValue);
    }

    return () -> defaultValue;
  }

  private FeatureFlag() {}
}
