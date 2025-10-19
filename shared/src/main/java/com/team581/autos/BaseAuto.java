package com.team581.autos;

import edu.wpi.first.math.geometry.Pose2d;

public interface BaseAuto {
  Pose2d getStartingPose();

  /** Returns the name of this auto. */
  default String name() {
    var className = this.getClass().getSimpleName();
    return className.substring(className.lastIndexOf('.') + 1);
  }
}
