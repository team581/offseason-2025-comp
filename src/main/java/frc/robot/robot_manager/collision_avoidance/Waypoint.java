package frc.robot.robot_manager.collision_avoidance;

import com.google.common.graph.MutableValueGraph;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.stream.Stream;

/**
 * These represent "waypoints" for collision avoidance to route through. These are NOT setpoints
 * that the robot uses, even though they may share a name and/or superstructure position. Collision
 * avoidance uses these as nodes within a graph to route from a current position to a goal position.
 */
public enum Waypoint {
  ALGAE_INTAKE_RIGHT(new SuperstructurePosition(3.0, 0.0)),
  LOLLIPOP_INTAKE_RIGHT(new SuperstructurePosition(0.0, 0.0)),
  STOWED(new SuperstructurePosition(55.0, -90.0)),
  LEFT_SAFE_STOWED_UP(new SuperstructurePosition(10.0, 90.0)),
  STOWED_UP(new SuperstructurePosition(0.0, 90.0)),
  HANDOFF(new SuperstructurePosition(41.12, -90.0)),
  L1_RIGHT(new SuperstructurePosition(0.0, 35.0)),
  L2_RIGHT(new SuperstructurePosition(7.2, 43.0)),
  L3_RIGHT(new SuperstructurePosition(22.0, 43.0)),
  L4_RIGHT(new SuperstructurePosition(46.0, 47.5)),
  L2_LEFT(new SuperstructurePosition(7.2, 180.0 - 43.0)),
  L3_LEFT(new SuperstructurePosition(22.0, 180.0 - 43.0)),
  L4_LEFT(new SuperstructurePosition(46.0, 180.0 - 47.5)),
  ALGAE_RIGHT(new SuperstructurePosition(60.0, 0.0)),
  ALGAE_LEFT(new SuperstructurePosition(60.0, 180.0)),
  ALGAE_L2_RIGHT(new SuperstructurePosition(24.0, 0.0)),
  ALGAE_L2_LEFT(new SuperstructurePosition(39.0, 180.0)),
  ALGAE_L3_RIGHT(new SuperstructurePosition(24.0, 0.0)),
  ALGAE_L3_LEFT(new SuperstructurePosition(39.0, 180.0));

  public final SuperstructurePosition position;

  Waypoint(SuperstructurePosition position) {
    this.position = position;
  }

  public double costFor(Waypoint other) {
    return position.costFor(other.position);
  }

  public static void log() {
    for (var waypoint : values()) {
      DogLog.log(
          "CollisionAvoidance/Waypoints/" + waypoint.toString(),
          waypoint.position.getTranslation());
    }
    DogLog.log(
        "CollisionAvoidance/Waypoints/All",
        Stream.of(values())
            .map(waypoint -> waypoint.position.getTranslation())
            .toArray(Translation2d[]::new));
  }

  /**
   * Find the closest waypoint to the given superstructure position.
   *
   * @param position The position of the superstructure.
   */
  public static Waypoint getClosest(SuperstructurePosition position) {
    Waypoint closestWaypoint = STOWED;
    double closestDistance = Double.MAX_VALUE;
    Translation2d point = position.getTranslation();

    for (int i = 0; Waypoint.values().length > i; i++) {

      Translation2d nodePoint = Waypoint.values()[i].position.getTranslation();

      double distanceFromNodeToPoint =
          Math.hypot(
              Math.pow(nodePoint.getX() - point.getX(), 2),
              Math.pow(nodePoint.getY() - point.getY(), 2));

      if (distanceFromNodeToPoint < closestDistance) {
        closestDistance = distanceFromNodeToPoint;
        closestWaypoint = Waypoint.values()[i];
      }
    }
    return closestWaypoint;
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
