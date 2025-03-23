package frc.robot.robot_manager.collision_avoidance;

import com.google.common.graph.ElementOrder;
import com.google.common.graph.MutableValueGraph;
import com.google.common.graph.ValueGraph;
import com.google.common.graph.ValueGraphBuilder;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public class CollisionAvoidance {

  private static final ValueGraph<Waypoint, WaypointEdge> graph = createGraph();

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

    var maybeResult = aStar(currentPosition, desiredPosition, obstructionKind);
    if (maybeResult.isPresent()) {
      return Optional.of(
          maybeResult.orElseThrow().get(0)); // go to the first waypoint in the list to goal
    }
    return Optional.empty();
  }

  private static ValueGraph<Waypoint, WaypointEdge> createGraph() {
    // Create an undirected value graph to represent safe motion between waypoints. Undirected
    // because if you can go from A to B, you can also go from B to A. Value graph because we want
    // to associate a cost with motion between different waypoints.
    MutableValueGraph<Waypoint, WaypointEdge> graph =
        ValueGraphBuilder.undirected().incidentEdgeOrder(ElementOrder.stable()).build();

    // Middle
    Waypoint.STOWED.canMoveToAlways(Waypoint.HANDOFF, graph);

    Waypoint.STOWED.canMoveToWhenLeftSafe(Waypoint.L3_LEFT, graph);
    Waypoint.STOWED.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.STOWED.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.STOWED.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.STOWED.canMoveToWhenRightSafe(Waypoint.ALGAE_RIGHT, graph);
    Waypoint.STOWED.canMoveToWhenLeftSafe(Waypoint.ALGAE_LEFT, graph);

    Waypoint.HANDOFF.canMoveToWhenLeftSafe(Waypoint.L3_LEFT, graph);
    Waypoint.HANDOFF.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.HANDOFF.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.HANDOFF.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.HANDOFF.canMoveToWhenLeftSafe(Waypoint.ALGAE_LEFT, graph);
    Waypoint.HANDOFF.canMoveToWhenRightSafe(Waypoint.ALGAE_RIGHT, graph);

    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.LOLLIPOP_INTAKE_LEFT, graph);
    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.ALGAE_INTAKE_LEFT, graph);
    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.L1_LEFT, graph);
    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.L2_LEFT, graph);
    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.L3_LEFT, graph);
    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.STOWED_UP.canMoveToWhenLeftSafe(Waypoint.ALGAE_LEFT, graph);
    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.L1_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.L2_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveToWhenRightSafe(Waypoint.ALGAE_RIGHT, graph);

    // Left side

    Waypoint.ALGAE_INTAKE_LEFT.canMoveToAlways(Waypoint.LOLLIPOP_INTAKE_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveToWhenLeftSafe(Waypoint.L1_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveToWhenLeftSafe(Waypoint.L2_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveToWhenLeftSafe(Waypoint.L3_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveToWhenLeftSafe(Waypoint.ALGAE_LEFT, graph);

    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveToWhenLeftSafe(Waypoint.L1_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveToWhenLeftSafe(Waypoint.L2_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveToWhenLeftSafe(Waypoint.L3_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveToWhenLeftSafe(Waypoint.ALGAE_LEFT, graph);

    Waypoint.L1_LEFT.canMoveToWhenLeftSafe(Waypoint.L2_LEFT, graph);
    Waypoint.L1_LEFT.canMoveToWhenLeftSafe(Waypoint.L3_LEFT, graph);
    Waypoint.L1_LEFT.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.L1_LEFT.canMoveToWhenLeftSafe(Waypoint.ALGAE_LEFT, graph);

    Waypoint.L2_LEFT.canMoveToWhenLeftSafe(Waypoint.L3_LEFT, graph);
    Waypoint.L2_LEFT.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.L2_LEFT.canMoveToWhenLeftSafe(Waypoint.ALGAE_LEFT, graph);

    Waypoint.L3_LEFT.canMoveToWhenLeftSafe(Waypoint.L4_LEFT, graph);
    Waypoint.L3_LEFT.canMoveToWhenLeftSafe(Waypoint.ALGAE_LEFT, graph);

    Waypoint.L4_LEFT.canMoveToWhenLeftSafe(Waypoint.ALGAE_LEFT, graph);

    // Right side
    Waypoint.L1_RIGHT.canMoveToWhenRightSafe(Waypoint.L2_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveToWhenRightSafe(Waypoint.ALGAE_RIGHT, graph);

    Waypoint.L2_RIGHT.canMoveToWhenRightSafe(Waypoint.L3_RIGHT, graph);
    Waypoint.L2_RIGHT.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.L2_RIGHT.canMoveToWhenRightSafe(Waypoint.ALGAE_RIGHT, graph);

    Waypoint.L3_RIGHT.canMoveToWhenRightSafe(Waypoint.L4_RIGHT, graph);
    Waypoint.L3_RIGHT.canMoveToWhenRightSafe(Waypoint.ALGAE_RIGHT, graph);

    Waypoint.L4_RIGHT.canMoveToWhenRightSafe(Waypoint.ALGAE_RIGHT, graph);

    // Visualize the generated graph
    GraphVisualizer.log(graph);
    Waypoint.log();

    return graph;
  }

  public static ArrayList<Waypoint> options(Waypoint waypoint, ObstructionKind obstructionKind) {
    return new ArrayList<Waypoint>(graph.adjacentNodes(waypoint));
  }

  /**
   * Returns an {@link Optional} containing the next {@link Waypoint} in the graph to go to. Returns
   * an empty Optional if there is no possible routing (impossible to avoid a collision or you are
   * at final waypoint).
   */
  private static ArrayList<Waypoint> reconstructPath(
      Map<Waypoint, Waypoint> cameFrom, Waypoint endWaypoint) {
    ArrayList<Waypoint> reversedTotalPath = new ArrayList<Waypoint>();
    ArrayList<Waypoint> totalPath = new ArrayList<Waypoint>();

    reversedTotalPath.add(endWaypoint);
    Waypoint current = endWaypoint;
    while (cameFrom.containsKey(current)) {
      current = cameFrom.get(current);
      reversedTotalPath.add(current);
      // How do i prepend an array list???
    }
    for (int i = 0; reversedTotalPath.size() > i; i++) {
      totalPath.add(reversedTotalPath.get(reversedTotalPath.size() - i - 1));
    }
    return totalPath;
  }

  static Optional<ArrayList<Waypoint>> aStar(
      SuperstructurePosition currentPosition,
      SuperstructurePosition desiredPosition,
      ObstructionKind obstructionKind) {
    Set<Waypoint> openSet = new HashSet<>(Set.of(Waypoint.getClosest(currentPosition)));

    Map<Waypoint, Waypoint> cameFrom = new HashMap<>();
    Map<Waypoint, Double> gscore = new HashMap<>();

    for (var waypoint : Waypoint.values()) {
      gscore.put(waypoint, Double.MAX_VALUE);
    }
    Waypoint startWaypoint = Waypoint.getClosest(currentPosition);
    Waypoint goalWaypoint = Waypoint.getClosest(desiredPosition);

    gscore.replace(startWaypoint, 0.0);
    Waypoint current = Waypoint.STOWED;
    while (!openSet.isEmpty()) {
      // current is equal to the waypoint in openset that has the smallest gscore
      double lowestGScore = Double.MAX_VALUE;

      for (Waypoint openPoint : openSet) {
        if (gscore.get(openPoint) < lowestGScore) {
          current = openPoint;
          lowestGScore = gscore.get(openPoint);
        }
      }
      if (current == goalWaypoint) {
        return Optional.of(reconstructPath(cameFrom, current));
      }
      openSet.remove(current);
      ArrayList<Waypoint> options = options(current, obstructionKind);

      for (Waypoint neighbor : options) {

        double tentativeGScore = gscore.get(current) + current.costFor(neighbor);
        if (tentativeGScore < gscore.get(neighbor)) {
          cameFrom.put(neighbor, current);
          gscore.replace(neighbor, tentativeGScore);
          if (!openSet.contains(neighbor)) {
            openSet.add(neighbor);
          }
        }
      }
    }
    return Optional.empty();
  }

  private CollisionAvoidance() {}
}
