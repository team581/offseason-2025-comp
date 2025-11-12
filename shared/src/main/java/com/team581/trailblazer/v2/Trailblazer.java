package com.team581.trailblazer.v2;

import com.team581.autos.Point;
import com.team581.trailblazer.LocalizationBase;
import com.team581.trailblazer.SwerveBase;

public class Trailblazer {
  public static AutoSegmentBuilder segment(Point... waypoints) {
    return new AutoSegmentBuilder();
  }

  private final SwerveBase swerve;
  private final LocalizationBase localization;

  public Trailblazer(SwerveBase swerve, LocalizationBase localization) {
    this.swerve = swerve;
    this.localization = localization;
  }

  public void followSegment(AutoSegment segment) {}

  public boolean atGoal() {}
}
