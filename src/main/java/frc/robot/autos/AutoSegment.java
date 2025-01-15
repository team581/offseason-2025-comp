package frc.robot.autos;

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
}
