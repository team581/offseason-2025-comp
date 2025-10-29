package com.team581.config;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DSOption {
  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("DSOptions");

  public static BooleanSubscriber of(String key, boolean defaultValue) {
    var entry = table.getBooleanTopic(key).getEntry(defaultValue);

    entry.set(defaultValue);

    return entry;
  }

  private DSOption() {}
}
