package frc.robot.robot_manager.collision_avoidance;

public record WaypointEdge(double cost) {
  public WaypointEdge(MotionWaypoint from, MotionWaypoint to) {
    this(from.costFor(to));
  }
}
