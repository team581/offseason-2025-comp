package com.team581.trailblazer.v2;

import com.team581.math.PoseErrorTolerance;

public class AutoSegmentBuilder {
  private AutoSegment build() {}

  public AutoSegmentBuilder withLinearConstraints(double maxVelocity, double maxAcceleration) {}

  public AutoSegmentBuilder withAngularConstraints(
      double maxAngularVelocity, double maxAngularAcceleration) {}

  public AutoSegment forever() {
    return build();
  }

  public AutoSegment untilFinished(PoseErrorTolerance tolerance) {
    return build();
  }
}
