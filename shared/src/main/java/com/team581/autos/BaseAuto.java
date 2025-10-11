package com.team581.autos;

import edu.wpi.first.math.geometry.Pose2d;

public abstract class BaseAuto {
  private final String autoName;

  protected BaseAuto() {
    var className = this.getClass().getSimpleName();
    autoName = className.substring(className.lastIndexOf('.') + 1);
  }

  public abstract Pose2d getStartingPose();

  /** Returns the name of this auto. */
  public String name() {
    return autoName;
  }
}
