package frc.robot.robot_manager.collision_avoidance;

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
      return Optional.of(maybeResult.orElseThrow().get(0)); // go to the first waypoint in the list to goal
    }
    return Optional.empty();
  }

  private static ValueGraph<Waypoint, WaypointEdge> createGraph() {
    // Create an undirected value graph to represent safe motion between waypoints. Undirected
    // because if you can go from A to B, you can also go from B to A. Value graph because we want
    // to associate a cost with motion between different waypoints.
    MutableValueGraph<Waypoint, WaypointEdge> graph = ValueGraphBuilder.undirected().build();

    // TODO(@ryanknj5): Add the full graph of safe motion
    // Middle
    Waypoint.STOWED.canMoveTo(Waypoint.L3_LEFT, graph);
    Waypoint.STOWED.canMoveTo(Waypoint.L3_RIGHT, graph);
    Waypoint.STOWED.canMoveTo(Waypoint.L4_LEFT, graph);
    Waypoint.STOWED.canMoveTo(Waypoint.L4_RIGHT, graph);
    Waypoint.STOWED.canMoveTo(Waypoint.ALGAE_LEFT, graph);
    Waypoint.STOWED.canMoveTo(Waypoint.ALGAE_RIGHT, graph);

    Waypoint.HANDOFF.canMoveTo(Waypoint.L3_LEFT, graph);
    Waypoint.HANDOFF.canMoveTo(Waypoint.L3_RIGHT, graph);
    Waypoint.HANDOFF.canMoveTo(Waypoint.L4_LEFT, graph);
    Waypoint.HANDOFF.canMoveTo(Waypoint.L4_RIGHT, graph);
    Waypoint.HANDOFF.canMoveTo(Waypoint.ALGAE_LEFT, graph);
    Waypoint.HANDOFF.canMoveTo(Waypoint.ALGAE_RIGHT, graph);

    Waypoint.STOWED_UP.canMoveTo(Waypoint.ALGAE_INTAKE_LEFT, graph);
    Waypoint.STOWED_UP.canMoveTo(Waypoint.LOLLIPOP_INTAKE_LEFT, graph);
    Waypoint.STOWED_UP.canMoveTo(Waypoint.L1_LEFT, graph);
    Waypoint.STOWED_UP.canMoveTo(Waypoint.L2_LEFT, graph);
    Waypoint.STOWED_UP.canMoveTo(Waypoint.L3_LEFT, graph);
    Waypoint.STOWED_UP.canMoveTo(Waypoint.L4_LEFT, graph);
    Waypoint.STOWED_UP.canMoveTo(Waypoint.ALGAE_LEFT, graph);
    Waypoint.STOWED_UP.canMoveTo(Waypoint.L1_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveTo(Waypoint.L2_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveTo(Waypoint.L3_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveTo(Waypoint.L4_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveTo(Waypoint.ALGAE_RIGHT, graph);

    // Left side

    Waypoint.ALGAE_INTAKE_LEFT.canMoveTo(Waypoint.LOLLIPOP_INTAKE_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveTo(Waypoint.L1_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveTo(Waypoint.L2_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveTo(Waypoint.L3_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveTo(Waypoint.L4_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveTo(Waypoint.ALGAE_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveTo(Waypoint.STOWED_UP, graph);

    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveTo(Waypoint.ALGAE_INTAKE_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveTo(Waypoint.L1_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveTo(Waypoint.L2_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveTo(Waypoint.L3_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveTo(Waypoint.L4_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveTo(Waypoint.ALGAE_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveTo(Waypoint.STOWED_UP, graph);

    Waypoint.L1_LEFT.canMoveTo(Waypoint.ALGAE_INTAKE_LEFT, graph);
    Waypoint.L1_LEFT.canMoveTo(Waypoint.LOLLIPOP_INTAKE_LEFT, graph);
    Waypoint.L1_LEFT.canMoveTo(Waypoint.L2_LEFT, graph);
    Waypoint.L1_LEFT.canMoveTo(Waypoint.L3_LEFT, graph);
    Waypoint.L1_LEFT.canMoveTo(Waypoint.L4_LEFT, graph);
    Waypoint.L1_LEFT.canMoveTo(Waypoint.ALGAE_LEFT, graph);
    Waypoint.L1_LEFT.canMoveTo(Waypoint.STOWED_UP, graph);

    Waypoint.L2_LEFT.canMoveTo(Waypoint.ALGAE_INTAKE_LEFT, graph);
    Waypoint.L2_LEFT.canMoveTo(Waypoint.LOLLIPOP_INTAKE_LEFT, graph);
    Waypoint.L2_LEFT.canMoveTo(Waypoint.L1_LEFT, graph);
    Waypoint.L2_LEFT.canMoveTo(Waypoint.L3_LEFT, graph);
    Waypoint.L2_LEFT.canMoveTo(Waypoint.L4_LEFT, graph);
    Waypoint.L2_LEFT.canMoveTo(Waypoint.ALGAE_LEFT, graph);
    Waypoint.L2_LEFT.canMoveTo(Waypoint.STOWED_UP, graph);

    Waypoint.L3_LEFT.canMoveTo(Waypoint.ALGAE_INTAKE_LEFT, graph);
    Waypoint.L3_LEFT.canMoveTo(Waypoint.LOLLIPOP_INTAKE_LEFT, graph);
    Waypoint.L3_LEFT.canMoveTo(Waypoint.L1_LEFT, graph);
    Waypoint.L3_LEFT.canMoveTo(Waypoint.L2_LEFT, graph);
    Waypoint.L3_LEFT.canMoveTo(Waypoint.L4_LEFT, graph);
    Waypoint.L3_LEFT.canMoveTo(Waypoint.ALGAE_LEFT, graph);
    Waypoint.L3_LEFT.canMoveTo(Waypoint.STOWED_UP, graph);
    Waypoint.L3_LEFT.canMoveTo(Waypoint.STOWED, graph);
    Waypoint.L3_LEFT.canMoveTo(Waypoint.HANDOFF, graph);

    Waypoint.L4_LEFT.canMoveTo(Waypoint.ALGAE_INTAKE_LEFT, graph);
    Waypoint.L4_LEFT.canMoveTo(Waypoint.LOLLIPOP_INTAKE_LEFT, graph);
    Waypoint.L4_LEFT.canMoveTo(Waypoint.L1_LEFT, graph);
    Waypoint.L4_LEFT.canMoveTo(Waypoint.L2_LEFT, graph);
    Waypoint.L4_LEFT.canMoveTo(Waypoint.L3_LEFT, graph);
    Waypoint.L4_LEFT.canMoveTo(Waypoint.ALGAE_LEFT, graph);
    Waypoint.L4_LEFT.canMoveTo(Waypoint.STOWED_UP, graph);
    Waypoint.L4_LEFT.canMoveTo(Waypoint.STOWED, graph);
    Waypoint.L4_LEFT.canMoveTo(Waypoint.HANDOFF, graph);

    Waypoint.ALGAE_LEFT.canMoveTo(Waypoint.ALGAE_INTAKE_LEFT, graph);
    Waypoint.ALGAE_LEFT.canMoveTo(Waypoint.LOLLIPOP_INTAKE_LEFT, graph);
    Waypoint.ALGAE_LEFT.canMoveTo(Waypoint.L1_LEFT, graph);
    Waypoint.ALGAE_LEFT.canMoveTo(Waypoint.L2_LEFT, graph);
    Waypoint.ALGAE_LEFT.canMoveTo(Waypoint.L3_LEFT, graph);
    Waypoint.ALGAE_LEFT.canMoveTo(Waypoint.L4_LEFT, graph);
    Waypoint.ALGAE_LEFT.canMoveTo(Waypoint.STOWED_UP, graph);
    Waypoint.ALGAE_LEFT.canMoveTo(Waypoint.STOWED, graph);
    Waypoint.ALGAE_LEFT.canMoveTo(Waypoint.HANDOFF, graph);

    // Right side
    Waypoint.L1_RIGHT.canMoveTo(Waypoint.L2_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveTo(Waypoint.L3_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveTo(Waypoint.L4_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveTo(Waypoint.ALGAE_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveTo(Waypoint.STOWED_UP, graph);

    Waypoint.L2_RIGHT.canMoveTo(Waypoint.L1_RIGHT, graph);
    Waypoint.L2_RIGHT.canMoveTo(Waypoint.L3_RIGHT, graph);
    Waypoint.L2_RIGHT.canMoveTo(Waypoint.L4_RIGHT, graph);
    Waypoint.L2_RIGHT.canMoveTo(Waypoint.ALGAE_RIGHT, graph);
    Waypoint.L2_RIGHT.canMoveTo(Waypoint.STOWED_UP, graph);

    Waypoint.L3_RIGHT.canMoveTo(Waypoint.L1_RIGHT, graph);
    Waypoint.L3_RIGHT.canMoveTo(Waypoint.L2_RIGHT, graph);
    Waypoint.L3_RIGHT.canMoveTo(Waypoint.L4_RIGHT, graph);
    Waypoint.L3_RIGHT.canMoveTo(Waypoint.ALGAE_RIGHT, graph);
    Waypoint.L3_RIGHT.canMoveTo(Waypoint.STOWED_UP, graph);
    Waypoint.L3_RIGHT.canMoveTo(Waypoint.STOWED, graph);
    Waypoint.L3_RIGHT.canMoveTo(Waypoint.HANDOFF, graph);

    Waypoint.L4_RIGHT.canMoveTo(Waypoint.L1_RIGHT, graph);
    Waypoint.L4_RIGHT.canMoveTo(Waypoint.L2_RIGHT, graph);
    Waypoint.L4_RIGHT.canMoveTo(Waypoint.L3_RIGHT, graph);
    Waypoint.L4_RIGHT.canMoveTo(Waypoint.ALGAE_RIGHT, graph);
    Waypoint.L4_RIGHT.canMoveTo(Waypoint.STOWED_UP, graph);
    Waypoint.L4_RIGHT.canMoveTo(Waypoint.STOWED, graph);
    Waypoint.L4_RIGHT.canMoveTo(Waypoint.HANDOFF, graph);

    Waypoint.ALGAE_RIGHT.canMoveTo(Waypoint.L1_RIGHT, graph);
    Waypoint.ALGAE_RIGHT.canMoveTo(Waypoint.L2_RIGHT, graph);
    Waypoint.ALGAE_RIGHT.canMoveTo(Waypoint.L3_RIGHT, graph);
    Waypoint.ALGAE_RIGHT.canMoveTo(Waypoint.L4_RIGHT, graph);
    Waypoint.ALGAE_RIGHT.canMoveTo(Waypoint.STOWED_UP, graph);
    Waypoint.ALGAE_RIGHT.canMoveTo(Waypoint.STOWED, graph);
    Waypoint.ALGAE_RIGHT.canMoveTo(Waypoint.HANDOFF, graph);

    // Visualize the generated graph
    GraphVisualizer.log(graph);

    return graph;
  }

  public static ArrayList<Waypoint> options(Waypoint waypoint, ObstructionKind obstructionKind) {
    System.out.println("optioning");
    return new ArrayList<Waypoint>(graph.adjacentNodes(waypoint));

    //     for (int o = 0; graph.adjacentNodes(waypoint).size() > o; o++) {
    // System.out.print("consider Options");
    // System.out.println(graph.adjacentNodes(waypoint));
    //       if (graph.adjacentNodes(waypoint).contains(Waypoint.values()[o])) {
    //         System.out.println("hass ekejsfdljflkdsjfijselfjsldfjdskljflkadsdjflk");
    //         options.add(Waypoint.values()[o]);
    //       }
    //     }
    //     System.out.println(options);
    //     return options;
  }

  /**
   * Returns an {@link Optional} containing the next {@link Waypoint} in the graph to go to. Returns
   * an empty Optional if there is no possible routing (impossible to avoid a collision or you are
   * at final waypoint).
   */
  private static ArrayList<Waypoint> reconstructPath(
      Map<Waypoint, Waypoint> cameFrom, Waypoint endWaypoint) {
    System.out.println("Starting reconsturction");
    ArrayList<Waypoint> reversedTotalPath = new ArrayList<Waypoint>();
    ArrayList<Waypoint> totalPath = new ArrayList<Waypoint>();

    reversedTotalPath.add(endWaypoint);
    Waypoint current = endWaypoint;
    while (cameFrom.containsKey(current)) {
      current = cameFrom.get(current);
      reversedTotalPath.add(current);
      // How do i prepend an array list???
      System.out.println("constructing path with came from");
    }
    System.out.println("going into for loop");
    for (int i = 0; reversedTotalPath.size() > i; i++) {
      System.out.println("adding reversedtotalpath size - i");
      System.out.println(reversedTotalPath.size() - i - 1);
      totalPath.add(reversedTotalPath.get(reversedTotalPath.size() - i - 1));
      System.out.println("IM Adding to total path:" + i);
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
    System.out.println("gscore:" + gscore);
    Waypoint startWaypoint = Waypoint.getClosest(currentPosition);
    Waypoint goalWaypoint = Waypoint.getClosest(desiredPosition);
    System.out.println("startWaypoint:" + startWaypoint);
    System.out.println("goalWaypoint:" + goalWaypoint);

    gscore.replace(startWaypoint, 0.0);
    Waypoint current = Waypoint.STOWED;
    while (!openSet.isEmpty()) {
      System.out.println("loop");
      // current is equal to the waypoint in openset that has the smallest gscore
      double lowestGScore = Double.MAX_VALUE;

      for (Waypoint openPoint : openSet) {
        if (gscore.get(openPoint) < lowestGScore) {
          current = openPoint;
          lowestGScore = gscore.get(openPoint);
        }
      }
      System.out.println("Lowest gscore in open set/CURRENT:" + current);
      if (current == goalWaypoint) {
        System.out.println("returning a star");
        System.out.println(cameFrom);
        return Optional.of(reconstructPath(cameFrom, current));
      }
      openSet.remove(current);
      ArrayList<Waypoint> options = options(current, obstructionKind);
      System.out.println("number of neighbors :" + options.size());
      System.out.println("all of neighbors :" + options);

      for (Waypoint neighbor : options) {
        System.out.println("Looking at  neighbors" + neighbor);

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
