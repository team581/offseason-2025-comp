package frc.robot.robot_manager.collision_avoidance;

import java.util.Optional;

public record WaypointEdge(
    /** The cost associated with the motion connecting the nodes on this edge. */
    double cost,
    /** Whether this motion is safe when the left side of the robot is obstructed. */
    boolean safeWhenLeftBlocked,
    /** Whether this motion is safe when the right side of the robot is obstructed. */
    boolean safeWhenRightBlocked,
    /**
     * The constraints for superstructure motion to use when moving along this edge. If empty,
     * regular motion magic parameters are used.
     */
    Optional<SuperstructureLimits> limits) {

  public static WaypointEdge rightUnblocked(Waypoint from, Waypoint to) {
    return new WaypointEdge(from, to, true, false, Optional.empty());
  }

  public static WaypointEdge rightUnblocked(
      Waypoint from, Waypoint to, SuperstructureLimits limits) {
    return new WaypointEdge(from, to, true, false, Optional.of(limits));
  }

  public static WaypointEdge leftUnblocked(Waypoint from, Waypoint to) {
    return new WaypointEdge(from, to, false, true, Optional.empty());
  }

  public static WaypointEdge leftUnblocked(
      Waypoint from, Waypoint to, SuperstructureLimits limits) {
    return new WaypointEdge(from, to, false, true, Optional.of(limits));
  }

  public static WaypointEdge alwaysSafe(Waypoint from, Waypoint to) {
    return new WaypointEdge(from, to, true, true, Optional.empty());
  }

  public static WaypointEdge alwaysSafe(Waypoint from, Waypoint to, SuperstructureLimits limits) {
    return new WaypointEdge(from, to, true, true, Optional.of(limits));
  }

  private WaypointEdge(
      Waypoint from,
      Waypoint to,
      boolean safeForLeftBlocked,
      boolean safeForRightBlocked,
      Optional<SuperstructureLimits> limits) {
    this(from.costFor(to), safeForLeftBlocked, safeForRightBlocked, limits);
  }

  public double getCost(ObstructionKind obstruction) {
    return switch (obstruction) {
      case LEFT_OBSTRUCTED -> safeWhenLeftBlocked ? cost : Double.MAX_VALUE;
      case RIGHT_OBSTRUCTED -> safeWhenRightBlocked ? cost : Double.MAX_VALUE;
      case NONE -> cost;
    };
  }
}
