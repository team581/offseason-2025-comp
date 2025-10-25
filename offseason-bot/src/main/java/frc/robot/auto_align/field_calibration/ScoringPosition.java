package frc.robot.auto_align.field_calibration;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auto_align.poses.ReefPipe;
import frc.robot.auto_align.poses.ReefPipeLevel;

public record ScoringPosition(ReefPipe pipe, boolean isRedAlliance) {
  public Pose2d getPose(ReefPipeLevel level) {
    return pipe.getPose(level, isRedAlliance);
  }
}
