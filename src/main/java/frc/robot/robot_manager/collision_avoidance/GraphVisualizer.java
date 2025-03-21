package frc.robot.robot_manager.collision_avoidance;

import com.google.common.graph.ValueGraph;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

public class GraphVisualizer {
  public static void log(ValueGraph<Waypoint, WaypointEdge> graph) {
    if (RobotBase.isReal()) {
      return;
    }

    var noObstructionContent =
        toMermaidStateDiagram("Collision avoidance (unobstructed)", graph, ObstructionKind.NONE);
    var leftObstructedContent =
        toMermaidStateDiagram(
            "Collision avoidance (left obstructed)", graph, ObstructionKind.LEFT_OBSTRUCTED);
    var rightObstructedContent =
        toMermaidStateDiagram(
            "Collision avoidance (right obstructed)", graph, ObstructionKind.RIGHT_OBSTRUCTED);

    // Write to files
    var rootDir = System.getProperty("user.dir");

    var docsDirPath = Path.of(rootDir, "docs", "collision_avoidance");
    var docsDir = docsDirPath.toFile();
    docsDir.mkdirs();

    var noObstructionPath = docsDirPath.resolve(Path.of("no_obstruction.md"));
    var leftObstructedPath = docsDirPath.resolve(Path.of("left_obstructed.md"));
    var rightObstructedPath = docsDirPath.resolve(Path.of("right_obstructed.md"));

    try {
      Files.writeString(noObstructionPath, noObstructionContent);
    } catch (IOException e) {
      e.printStackTrace();
      DogLog.logFault("Couldn't write collision avoidance graph (unobstructed)");
    }

    try {
      Files.writeString(leftObstructedPath, leftObstructedContent);
    } catch (IOException e) {
      e.printStackTrace();
      DogLog.logFault("Couldn't write collision avoidance graph (left obstructed)");
    }

    try {
      Files.writeString(rightObstructedPath, rightObstructedContent);
    } catch (IOException e) {
      e.printStackTrace();
      DogLog.logFault("Couldn't write collision avoidance graph (right obstructed)");
    }
  }

  private static String toMermaidStateDiagram(
      String title, ValueGraph<Waypoint, WaypointEdge> graph, ObstructionKind obstruction) {
    var sb = new StringBuilder();

    sb.append("# ");
    sb.append(title);
    sb.append("\n\n```mermaid\nstateDiagram-v2\n");

    for (var pair : graph.edges()) {
      var edge = graph.edgeValue(pair).orElseThrow();

      // U -> V
      sb.append(pair.nodeU().toString());
      sb.append(" --> ");
      sb.append(pair.nodeV().toString());
      sb.append(" : ");
      sb.append(edge.getCost(obstruction));
      sb.append('\n');
      // V -> U
      sb.append(pair.nodeV().toString());
      sb.append(" --> ");
      sb.append(pair.nodeU().toString());
      sb.append(" : ");
      sb.append(edge.getCost(obstruction));
      sb.append('\n');
    }

    sb.append("```\n");

    return sb.toString();
  }

  private GraphVisualizer() {}
}
