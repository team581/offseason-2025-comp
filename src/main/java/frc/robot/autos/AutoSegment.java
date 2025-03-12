package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autos.constraints.AutoConstraintOptions;
import java.util.List;

/**
 * A segment is a path (a continuous set of {@link AutoPoint points}) that the roobt will follow.
 */
public class AutoSegment {
  public final List<AutoPoint> points;

  /**
   * Constraints to apply to any points that don't have their own constraints specified. If a point
   * specifies its own constraints, this field will be ignored.
   */
  public final AutoConstraintOptions defaultConstraints;

  public AutoSegment(AutoConstraintOptions defaultConstraints, List<AutoPoint> points) {
    this.defaultConstraints = defaultConstraints;
    this.points = points;
  }

  public AutoSegment(AutoConstraintOptions defaultConstraints, AutoPoint... points) {
    this(defaultConstraints, List.of(points));
  }

  public AutoSegment(AutoPoint... points) {
    this(new AutoConstraintOptions(), points);
  }

  public AutoSegment pathflipped() {
    return new AutoSegment(
        defaultConstraints, points.stream().map(AutoPoint::pathflipped).toList());
  }

  /**
   * Get the remaining distance from the robot's current pose to the end of the segment.
   *
   * @param robotPose The current pose of the robot.
   * @param currentIndex The current index of the segment.
   */
  public double getRemainingDistance(Pose2d robotPose, int currentIndex) {
    var remainingPoints = points.subList(currentIndex, points.size());

    var distance = 0.0;

    for (var i = 0; i < remainingPoints.size(); i++) {
      var previous = i == 0 ? robotPose : remainingPoints.get(i - 1).poseSupplier.get();
      var current = remainingPoints.get(i).poseSupplier.get();

      distance += previous.getTranslation().getDistance(current.getTranslation());
    }

    return distance;
  }
}
