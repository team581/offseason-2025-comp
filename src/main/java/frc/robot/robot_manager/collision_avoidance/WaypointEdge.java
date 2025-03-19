package frc.robot.robot_manager.collision_avoidance;

public record WaypointEdge(double cost) {
  public WaypointEdge(Waypoint from, Waypoint to) {
    this(from.costFor(to));
  }
}
