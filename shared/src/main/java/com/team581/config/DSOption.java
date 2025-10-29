package com.team581.config;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DSOption {
  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("DSOptions");

  public static BooleanSubscriber of(String key, boolean defaultValue) {
    var topic = table.getBooleanTopic(key);

    topic.publish().setDefault(defaultValue);

    return topic.subscribe(defaultValue);
  }

  private DSOption() {}
}
