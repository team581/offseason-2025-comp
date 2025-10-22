package com.team581.config;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;

public class DSOption {
  private final BooleanSubscriber booleanSub;
 public DSOption(String key, boolean defaultValue) {

    var table = NetworkTableInstance.getDefault()
        .getTable("DSOptions");
    BooleanTopic booleanTopic = table.getBooleanTopic(key);

    booleanTopic.publish();
    this.booleanSub = booleanTopic.subscribe(defaultValue);
  }

  public boolean getAsBoolean() {
    return booleanSub.getAsBoolean();
  }
}
