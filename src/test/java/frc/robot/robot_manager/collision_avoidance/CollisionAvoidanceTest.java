package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayList;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {
  @Test
  public void astarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(0, 90), new SuperstructurePosition(50, 0), null);
    ArrayList<Waypoint> expected = new ArrayList<Waypoint>();
    expected.add(Waypoint.ALGAE_INTAKE_LEFT);
    expected.add(Waypoint.ALGAE_INTAKE_LEFT);

    assertEquals(expected, result);
  }
}
