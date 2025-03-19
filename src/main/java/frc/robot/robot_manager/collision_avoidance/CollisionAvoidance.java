package frc.robot.robot_manager.collision_avoidance;

import com.google.common.graph.MutableValueGraph;
import com.google.common.graph.ValueGraph;
import com.google.common.graph.ValueGraphBuilder;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.Optional;

public class CollisionAvoidance {
  private static ValueGraph<MotionWaypoint, WaypointEdge> createGraph(
      ObstructionKind obstructions) {
    // Create an undirected value graph to represent safe motion between waypoints. Undirected
    // because if you can go from A to B, you can also go from B to A. Value graph because we want
    // to associate a cost with motion between different waypoints.
    MutableValueGraph<MotionWaypoint, WaypointEdge> graph = ValueGraphBuilder.undirected().build();

    // TODO(@ryanknj5): Add the full graph of safe motion

    // Left side
    graph.addNode(MotionWaypoint.LOLLIPOP_INTAKE_LEFT);
    graph.addNode(MotionWaypoint.ALGAE_INTAKE_LEFT);

    MotionWaypoint.LOLLIPOP_INTAKE_LEFT.canMoveTo(MotionWaypoint.ALGAE_INTAKE_LEFT, graph);

    // Right side
    graph.addNode(MotionWaypoint.LOLLIPOP_INTAKE_RIGHT);
    graph.addNode(MotionWaypoint.ALGAE_INTAKE_RIGHT);

    MotionWaypoint.ALGAE_INTAKE_RIGHT.canMoveTo(MotionWaypoint.LOLLIPOP_INTAKE_RIGHT, graph);

    // Return an immutable version now that we have finished constructing the graph
    return ValueGraphBuilder.from(graph).immutable().build();
  }

  /** Graph of safe motion when there aren't any special obstructions. */
  private final ValueGraph<MotionWaypoint, WaypointEdge> notObstructedGraph =
      createGraph(ObstructionKind.NONE);

  /** Graph of safe motion when there are obstructions (the reef) on the left side of the robot. */
  private final ValueGraph<MotionWaypoint, WaypointEdge> leftObstructedGraph =
      createGraph(ObstructionKind.LEFT_OBSTRUCTED);

  /** Graph of safe motion when there are obstructions (the reef) on the right side of the robot. */
  private final ValueGraph<MotionWaypoint, WaypointEdge> rightObstructedGraph =
      createGraph(ObstructionKind.RIGHT_OBSTRUCTED);

  /**
   * Returns an {@link Optional} containing the next {@link MotionWaypoint} in the graph to go to.
   * Returns an empty Optional if there is no possible routing (impossible to avoid a collision or
   * you are at final waypoint).
   *
   * @param currentPosition The current position of the superstructure.
   * @param desiredPosition The desired position of the superstructure.
   * @param obstructionKind Additional constraints based on robot position.
   */
  public Optional<MotionWaypoint> route(
      SuperstructurePosition currentPosition,
      SuperstructurePosition desiredPosition,
      ObstructionKind obstructionKind) {
    var graph = getGraph(obstructionKind);

    // TODO(@ryanknj5): Implement
    return Optional.empty();
  }

  private ValueGraph<MotionWaypoint, WaypointEdge> getGraph(ObstructionKind obstructionKind) {
    return switch (obstructionKind) {
      case LEFT_OBSTRUCTED -> leftObstructedGraph;
      case RIGHT_OBSTRUCTED -> rightObstructedGraph;
      case NONE -> notObstructedGraph;
    };
  }
}
