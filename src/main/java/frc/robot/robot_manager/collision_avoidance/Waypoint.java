package frc.robot.robot_manager.collision_avoidance;

import com.google.common.graph.MutableValueGraph;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.robot_manager.SuperstructurePosition;

/**
 * These represent "waypoints" for collision avoidance to route through. These are NOT setpoints
 * that the robot uses, even though they may share a name and/or superstructure position. Collision
 * avoidance uses these as nodes within a graph to route from a current position to a goal position.
 */
public enum Waypoint {
  // TODO(@ryanknj5): Add more waypoints (can estimate/vibe out the positions)
  ALGAE_INTAKE_LEFT(new SuperstructurePosition(3, 190)),
  LOLLIPOP_INTAKE_LEFT(new SuperstructurePosition(0, 180)),
  STOWED(new SuperstructurePosition(20, -90)),
  STOWED_UP(new SuperstructurePosition(0, 90)),
  HANDOFF(new SuperstructurePosition(24, -90)),
  L1_RIGHT(new SuperstructurePosition(5, 0)),
  L2_RIGHT(new SuperstructurePosition(20, 0)),
  L3_RIGHT(new SuperstructurePosition(40, 0)),
  L4_RIGHT(new SuperstructurePosition(55, 0)),
  L1_LEFT(new SuperstructurePosition(5, 180)),
  L2_LEFT(new SuperstructurePosition(20, 180)),
  L3_LEFT(new SuperstructurePosition(40, 180)),
  L4_LEFT(new SuperstructurePosition(55, 180)),
  ALGAE_RIGHT(new SuperstructurePosition(55, 0)),
  ALGAE_LEFT(new SuperstructurePosition(55, 180));

  public final SuperstructurePosition position;

  Waypoint(SuperstructurePosition position) {
    this.position = position;
  }

  public double costFor(Waypoint other) {
    return position.costFor(other.position);
  }

  private static final double WRIST_LENGTH = 15;

  /**
   * Find the closest waypoint to the given superstructure position.
   *
   * @param position The position of the superstructure.
   */
  public static Waypoint getClosest(SuperstructurePosition position) {
    Waypoint closestWaypoint = STOWED;
    double closestDistance = Double.MAX_VALUE;
    Translation2d point =
        new Translation2d(0, position.elevatorHeight())
            .plus(new Translation2d(WRIST_LENGTH, position.armAngle()));

    for (int i = 0; Waypoint.values().length > i; i++) {

      Translation2d nodePoint =
          new Translation2d(0, Waypoint.values()[i].position.elevatorHeight())
              .plus(new Translation2d(WRIST_LENGTH, Waypoint.values()[i].position.armAngle()));

      double distanceFromNodeToPoint =
          Math.sqrt(
              Math.pow(nodePoint.getX() - point.getX(), 2)
                  + Math.pow(nodePoint.getY() - point.getY(), 2));

      if (distanceFromNodeToPoint < closestDistance) {
        closestDistance = distanceFromNodeToPoint;
        closestWaypoint = Waypoint.values()[i];
      }
    }
    return closestWaypoint;
  }

  /**
   * @deprecated Use {@link #canMoveToAlways(Waypoint, MutableValueGraph)} instead.
   */
  @Deprecated(forRemoval = true)
  public void canMoveTo(Waypoint other, MutableValueGraph<Waypoint, WaypointEdge> graph) {
    canMoveToAlways(other, graph);
  }

  public void canMoveToAlways(Waypoint other, MutableValueGraph<Waypoint, WaypointEdge> graph) {
    var existingEdge = graph.putEdgeValue(this, other, WaypointEdge.alwaysSafe(this, other));

    if (existingEdge != null) {
      throw new IllegalStateException("Redundant edge connecting " + this + " to " + other);
    }
  }

  public void canMoveToWhenLeftSafe(
      Waypoint other, MutableValueGraph<Waypoint, WaypointEdge> graph) {
    var existingEdge = graph.putEdgeValue(this, other, WaypointEdge.leftUnblocked(this, other));

    if (existingEdge != null) {
      throw new IllegalStateException("Redundant edge connecting " + this + " to " + other);
    }
  }

  public void canMoveToWhenRightSafe(
      Waypoint other, MutableValueGraph<Waypoint, WaypointEdge> graph) {
    var existingEdge = graph.putEdgeValue(this, other, WaypointEdge.rightUnblocked(this, other));

    if (existingEdge != null) {
      throw new IllegalStateException("Redundant edge connecting " + this + " to " + other);
    }
  }
}
