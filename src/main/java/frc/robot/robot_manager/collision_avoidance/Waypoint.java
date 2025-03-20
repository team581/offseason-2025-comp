package frc.robot.robot_manager.collision_avoidance;

import com.google.common.graph.MutableValueGraph;
import frc.robot.robot_manager.SuperstructurePosition;

/**
 * These represent "waypoints" for collision avoidance to route through. These are NOT setpoints
 * that the robot uses, even though they may share a name and/or superstructure position. Collision
 * avoidance uses these as nodes within a graph to route from a current position to a goal position.
 */
public enum Waypoint {
  // TODO(@ryanknj5): Add more waypoints (can estimate/vibe out the positions)
  ALGAE_INTAKE_LEFT(new SuperstructurePosition(3, 190)),
  ALGAE_INTAKE_RIGHT(new SuperstructurePosition(3, -10)),
  LOLLIPOP_INTAKE_LEFT(new SuperstructurePosition(0, 0)),
  LOLLIPOP_INTAKE_RIGHT(new SuperstructurePosition(0, 180));

  public final SuperstructurePosition position;

  Waypoint(SuperstructurePosition position) {
    this.position = position;
  }

  public double costFor(Waypoint other) {
    return position.costFor(other.position);
  }

  /**
   * Find the closest waypoint to the given superstructure position.
   *
   * @param position The position of the superstructure.
   */
  public static Waypoint getClosest(SuperstructurePosition position) {
    // TODO(@ryanknj5): Implement
    return Waypoint.ALGAE_INTAKE_LEFT;
  }

  public void canMoveToAlways(Waypoint other, MutableValueGraph<Waypoint, WaypointEdge> graph) {
    graph.putEdgeValue(this, other, WaypointEdge.alwaysSafe(this, other));
  }

  public void canMoveToWhenLeftSafe(
      Waypoint other, MutableValueGraph<Waypoint, WaypointEdge> graph) {
    graph.putEdgeValue(this, other, WaypointEdge.leftUnblocked(this, other));
  }

  public void canMoveToWhenRightSafe(
      Waypoint other, MutableValueGraph<Waypoint, WaypointEdge> graph) {
    graph.putEdgeValue(this, other, WaypointEdge.rightUnblocked(this, other));
  }
}
