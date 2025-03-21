package frc.robot.robot_manager.collision_avoidance;

import com.google.common.graph.MutableValueGraph;
import com.google.common.graph.ValueGraph;
import com.google.common.graph.ValueGraphBuilder;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.Optional;

public class CollisionAvoidance {
  private static final ValueGraph<Waypoint, WaypointEdge> graph = createGraph();

  private static ValueGraph<Waypoint, WaypointEdge> createGraph() {
    // Create an undirected value graph to represent safe motion between waypoints. Undirected
    // because if you can go from A to B, you can also go from B to A. Value graph because we want
    // to associate a cost with motion between different waypoints.
    MutableValueGraph<Waypoint, WaypointEdge> graph = ValueGraphBuilder.undirected().build();

    // TODO(@ryanknj5): Add the full graph of safe motion

    // Left side
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveToAlways(Waypoint.ALGAE_INTAKE_LEFT, graph);

    // Right side
    Waypoint.ALGAE_INTAKE_RIGHT.canMoveToAlways(Waypoint.LOLLIPOP_INTAKE_RIGHT, graph);

    // Visualize the generated graph
    GraphVisualizer.log(graph);

    // Return an immutable version now that we have finished constructing the graph
    return ValueGraphBuilder.from(graph).immutable().build();
  }

  /**
   * Returns an {@link Optional} containing the next {@link Waypoint} in the graph to go to. Returns
   * an empty Optional if there is no possible routing (impossible to avoid a collision or you are
   * at final waypoint).
   *
   * @param currentPosition The current position of the superstructure.
   * @param desiredPosition The desired position of the superstructure.
   * @param obstructionKind Additional constraints based on robot position.
   */
  public static Optional<Waypoint> route(
      SuperstructurePosition currentPosition,
      SuperstructurePosition desiredPosition,
      ObstructionKind obstructionKind) {
    // TODO(@ryanknj5): Implement

    return Optional.empty();
  }

  private CollisionAvoidance() {}
}
