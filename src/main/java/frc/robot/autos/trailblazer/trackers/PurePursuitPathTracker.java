package frc.robot.autos.trailblazer.trackers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.autos.trailblazer.AutoPoint;
import java.util.List;

// TODO: Implement https://github.com/team581/2024-offseason-bot/issues/95
public class PurePursuitPathTracker implements PathTracker {
  @Override
  public void resetAndSetPoints(List<AutoPoint> points) {}

  @Override
  public void updateRobotState(Pose2d currentPose, ChassisSpeeds currentFieldRelativeRobotSpeeds) {}

  @Override
  public Pose2d getTargetPose() {
    return null;
  }

  @Override
  public int getCurrentPointIndex() {
    return 0;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
