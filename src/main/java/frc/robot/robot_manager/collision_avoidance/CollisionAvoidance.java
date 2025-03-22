package frc.robot.robot_manager.collision_avoidance;

import com.google.common.graph.MutableValueGraph;
import com.google.common.graph.ValueGraph;
import com.google.common.graph.ValueGraphBuilder;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public class CollisionAvoidance {

  private static final double WRIST_LENGTH = 15; // NOT REAL

  private static ValueGraph<Waypoint, WaypointEdge> createGraph(ObstructionKind obstructions) {
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
    Waypoint.STOWED_UP.canMoveTo(Waypoint.STOWED_UP, graph);
    Waypoint.STOWED_UP.canMoveTo(Waypoint.L1_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveTo(Waypoint.L2_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveTo(Waypoint.L3_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveTo(Waypoint.L4_RIGHT, graph);
    Waypoint.STOWED_UP.canMoveTo(Waypoint.ALGAE_RIGHT, graph);

    // Left side

    Waypoint.ALGAE_INTAKE_LEFT.canMoveTo(Waypoint.ALGAE_INTAKE_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveTo(Waypoint.LOLLIPOP_INTAKE_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveTo(Waypoint.L1_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveTo(Waypoint.L2_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveTo(Waypoint.L3_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveTo(Waypoint.L4_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveTo(Waypoint.ALGAE_LEFT, graph);
    Waypoint.ALGAE_INTAKE_LEFT.canMoveTo(Waypoint.STOWED_UP, graph);

    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveTo(Waypoint.ALGAE_INTAKE_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveTo(Waypoint.LOLLIPOP_INTAKE_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveTo(Waypoint.L1_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveTo(Waypoint.L2_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveTo(Waypoint.L3_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveTo(Waypoint.L4_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveTo(Waypoint.ALGAE_LEFT, graph);
    Waypoint.LOLLIPOP_INTAKE_LEFT.canMoveTo(Waypoint.STOWED_UP, graph);

    Waypoint.L1_LEFT.canMoveTo(Waypoint.ALGAE_INTAKE_LEFT, graph);
    Waypoint.L1_LEFT.canMoveTo(Waypoint.LOLLIPOP_INTAKE_LEFT, graph);
    Waypoint.L1_LEFT.canMoveTo(Waypoint.L1_LEFT, graph);
    Waypoint.L1_LEFT.canMoveTo(Waypoint.L2_LEFT, graph);
    Waypoint.L1_LEFT.canMoveTo(Waypoint.L3_LEFT, graph);
    Waypoint.L1_LEFT.canMoveTo(Waypoint.L4_LEFT, graph);
    Waypoint.L1_LEFT.canMoveTo(Waypoint.ALGAE_LEFT, graph);
    Waypoint.L1_LEFT.canMoveTo(Waypoint.STOWED_UP, graph);

    Waypoint.L2_LEFT.canMoveTo(Waypoint.ALGAE_INTAKE_LEFT, graph);
    Waypoint.L2_LEFT.canMoveTo(Waypoint.LOLLIPOP_INTAKE_LEFT, graph);
    Waypoint.L2_LEFT.canMoveTo(Waypoint.L1_LEFT, graph);
    Waypoint.L2_LEFT.canMoveTo(Waypoint.L2_LEFT, graph);
    Waypoint.L2_LEFT.canMoveTo(Waypoint.L3_LEFT, graph);
    Waypoint.L2_LEFT.canMoveTo(Waypoint.L4_LEFT, graph);
    Waypoint.L2_LEFT.canMoveTo(Waypoint.ALGAE_LEFT, graph);
    Waypoint.L2_LEFT.canMoveTo(Waypoint.STOWED_UP, graph);

    Waypoint.L3_LEFT.canMoveTo(Waypoint.ALGAE_INTAKE_LEFT, graph);
    Waypoint.L3_LEFT.canMoveTo(Waypoint.LOLLIPOP_INTAKE_LEFT, graph);
    Waypoint.L3_LEFT.canMoveTo(Waypoint.L1_LEFT, graph);
    Waypoint.L3_LEFT.canMoveTo(Waypoint.L2_LEFT, graph);
    Waypoint.L3_LEFT.canMoveTo(Waypoint.L3_LEFT, graph);
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
    Waypoint.L4_LEFT.canMoveTo(Waypoint.L4_LEFT, graph);
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
    Waypoint.ALGAE_LEFT.canMoveTo(Waypoint.ALGAE_LEFT, graph);
    Waypoint.ALGAE_LEFT.canMoveTo(Waypoint.STOWED_UP, graph);
    Waypoint.ALGAE_LEFT.canMoveTo(Waypoint.STOWED, graph);
    Waypoint.ALGAE_LEFT.canMoveTo(Waypoint.HANDOFF, graph);

    // Right side
    Waypoint.L1_RIGHT.canMoveTo(Waypoint.L1_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveTo(Waypoint.L2_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveTo(Waypoint.L3_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveTo(Waypoint.L4_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveTo(Waypoint.ALGAE_RIGHT, graph);
    Waypoint.L1_RIGHT.canMoveTo(Waypoint.STOWED_UP, graph);

    Waypoint.L2_RIGHT.canMoveTo(Waypoint.L1_RIGHT, graph);
    Waypoint.L2_RIGHT.canMoveTo(Waypoint.L2_RIGHT, graph);
    Waypoint.L2_RIGHT.canMoveTo(Waypoint.L3_RIGHT, graph);
    Waypoint.L2_RIGHT.canMoveTo(Waypoint.L4_RIGHT, graph);
    Waypoint.L2_RIGHT.canMoveTo(Waypoint.ALGAE_RIGHT, graph);
    Waypoint.L2_RIGHT.canMoveTo(Waypoint.STOWED_UP, graph);

    Waypoint.L3_RIGHT.canMoveTo(Waypoint.L1_RIGHT, graph);
    Waypoint.L3_RIGHT.canMoveTo(Waypoint.L2_RIGHT, graph);
    Waypoint.L3_RIGHT.canMoveTo(Waypoint.L3_RIGHT, graph);
    Waypoint.L3_RIGHT.canMoveTo(Waypoint.L4_RIGHT, graph);
    Waypoint.L3_RIGHT.canMoveTo(Waypoint.ALGAE_RIGHT, graph);
    Waypoint.L3_RIGHT.canMoveTo(Waypoint.STOWED_UP, graph);
    Waypoint.L3_RIGHT.canMoveTo(Waypoint.STOWED, graph);
    Waypoint.L3_RIGHT.canMoveTo(Waypoint.HANDOFF, graph);

    Waypoint.L4_RIGHT.canMoveTo(Waypoint.L1_RIGHT, graph);
    Waypoint.L4_RIGHT.canMoveTo(Waypoint.L2_RIGHT, graph);
    Waypoint.L4_RIGHT.canMoveTo(Waypoint.L3_RIGHT, graph);
    Waypoint.L4_RIGHT.canMoveTo(Waypoint.L4_RIGHT, graph);
    Waypoint.L4_RIGHT.canMoveTo(Waypoint.ALGAE_RIGHT, graph);
    Waypoint.L4_RIGHT.canMoveTo(Waypoint.STOWED_UP, graph);
    Waypoint.L4_RIGHT.canMoveTo(Waypoint.STOWED, graph);
    Waypoint.L4_RIGHT.canMoveTo(Waypoint.HANDOFF, graph);

    Waypoint.ALGAE_RIGHT.canMoveTo(Waypoint.L1_RIGHT, graph);
    Waypoint.ALGAE_RIGHT.canMoveTo(Waypoint.L2_RIGHT, graph);
    Waypoint.ALGAE_RIGHT.canMoveTo(Waypoint.L3_RIGHT, graph);
    Waypoint.ALGAE_RIGHT.canMoveTo(Waypoint.L4_RIGHT, graph);
    Waypoint.ALGAE_RIGHT.canMoveTo(Waypoint.ALGAE_RIGHT, graph);
    Waypoint.ALGAE_RIGHT.canMoveTo(Waypoint.STOWED_UP, graph);
    Waypoint.ALGAE_RIGHT.canMoveTo(Waypoint.STOWED, graph);
    Waypoint.ALGAE_RIGHT.canMoveTo(Waypoint.HANDOFF, graph);

    // Return an immutable version now that we have finished constructing the graph
    return ValueGraphBuilder.from(graph).immutable().build();
  }

  public static ArrayList<Waypoint> options(Waypoint waypoint, ObstructionKind obstructionKind) {
    var graph = CollisionAvoidance.getGraph(obstructionKind);
    ArrayList<Waypoint> options = new ArrayList<Waypoint>();

    for (int o = 0; Waypoint.values().length > o; o++) {
      if (graph.hasEdgeConnecting(waypoint, Waypoint.values()[o])) {
        options.add(Waypoint.values()[o]);
      }
    }
    return options;
  }

  /**
   * Returns an {@link Optional} containing the next {@link Waypoint} in the graph to go to. Returns
   * an empty Optional if there is no possible routing (impossible to avoid a collision or you are
   * at final waypoint).
   */
  public static ArrayList<Waypoint> reconstructPath(
      Map<Waypoint, Waypoint> cameFrom, Waypoint endWaypoint) {
    ArrayList<Waypoint> reversedTotalPath = new ArrayList<Waypoint>();
    ArrayList<Waypoint> totalPath = new ArrayList<Waypoint>();

    Waypoint current;
    reversedTotalPath.add(endWaypoint);
    while (cameFrom.containsKey(endWaypoint)) {
      current = cameFrom.get(endWaypoint);
      reversedTotalPath.add(current);
      // How do i prepend an array list???

    }
    for (int i = 0; reversedTotalPath.size() > i; i++) {
      totalPath.add(reversedTotalPath.get(reversedTotalPath.size() - i));
    }
    return totalPath;
  }

  public static Optional<ArrayList<Waypoint>> aStar(
      SuperstructurePosition currentPosition,
      SuperstructurePosition desiredPosition,
      ObstructionKind obstructionKind) {

    var graph = getGraph(obstructionKind);
    Set<Waypoint> openSet = Set.of(Waypoint.getClosest(currentPosition));

    Map<Waypoint, Waypoint> cameFrom = Map.ofEntries();
    Map<Waypoint, Double> gscore = Map.ofEntries();

    for (int w = 0; Waypoint.values().length > 0; w++) {
      gscore.put(Waypoint.values()[w], Double.MAX_VALUE);
      // cameFrom.put(Waypoint.values()[w], null);
    }
    Waypoint startWaypoint = Waypoint.getClosest(currentPosition);
    Waypoint goalWaypoint = Waypoint.getClosest(desiredPosition);
    gscore.replace(startWaypoint, 0.0);

    while (!openSet.isEmpty()) {
      // current is equal to the waypoint in openset that has the smallest gscore
      Waypoint current = Waypoint.STOWED;
      double lowestGScore = Double.MAX_VALUE;
      for (int i = 0; gscore.size() > 0; i++) {

        if (openSet.contains(Waypoint.values()[i])
            && gscore.get(Waypoint.values()[i]) < lowestGScore) {
          current = Waypoint.values()[i];
          lowestGScore = gscore.get(Waypoint.values()[i]);
        }
      }

      if (current == goalWaypoint) {
        return Optional.of(reconstructPath(cameFrom, current)); // TODO make recontstrut path
      }
      openSet.remove(current);
      for (int i = 0; options(current, ObstructionKind.NONE).size() > i; i++) {
        Waypoint neighbor = options(current, ObstructionKind.NONE).get(i);

        double tentativeGScore = gscore.get(current) + current.costFor(neighbor);
        if (tentativeGScore < gscore.get(neighbor)) {
          cameFrom.put(neighbor, current);
          gscore.replace(neighbor, tentativeGScore);
        }
        if (!openSet.contains(neighbor)) {
          openSet.add(neighbor);
        }
      }
    }
    return Optional.empty();
  }

  private static ValueGraph<Waypoint, WaypointEdge> getGraph(ObstructionKind obstructionKind) {
    return createGraph(obstructionKind);
    // return switch (obstructionKind) {
    //   case LEFT_OBSTRUCTED -> createGraph(obstructionKind);
    //   case RIGHT_OBSTRUCTED -> createGraph(obstructionKind);
    //   case NONE -> createGraph(obstructionKind);
    // };
  }

  private CollisionAvoidance() {}
}
