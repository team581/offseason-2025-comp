package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.Optional;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {
  @Test
  void testPositionToTranslation() {
    var result = CollisionAvoidance.positionToTranslation(new SuperstructurePosition(0, 0));
    var expectedResult = new Translation2d(19, 0);
    assertEquals(expectedResult, result);
  }

  @Test
  void testInZone() {
    SuperstructurePosition current = new SuperstructurePosition(0, 90);
    CollisionBox box = CollisionBox.BOX_2;
    var result = CollisionAvoidance.getZone(current);
    assertEquals(box, result);
  }

  @Test
  void testGetZoneOutofBounds() {
    SuperstructurePosition current = new SuperstructurePosition(100, 45);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_5;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone1() {
    SuperstructurePosition current = new SuperstructurePosition(25, 180);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_1;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone2() {
    SuperstructurePosition current = new SuperstructurePosition(0, 180);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_2;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone3() {
    SuperstructurePosition current = new SuperstructurePosition(0, 45);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_3;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone4() {
    SuperstructurePosition current = new SuperstructurePosition(0, 0);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_3;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone6() {
    SuperstructurePosition current = new SuperstructurePosition(67, 45);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_5;
    assertEquals(expectedResult, result);
  }

  @Test
  void testGetZone7() {
    SuperstructurePosition current = new SuperstructurePosition(67, 135);
    var result = CollisionAvoidance.getZone(current);
    var expectedResult = CollisionBox.BOX_6;
    assertEquals(expectedResult, result);
  }

  @Test
  void testPlanNoCollisionsLow() {
    SuperstructurePosition current = new SuperstructurePosition(0, 150);
    SuperstructurePosition goal = new SuperstructurePosition(20, 150);
    SuperstructurePosition expectedResult = new SuperstructurePosition(13, 180);
    var result = CollisionAvoidance.plan(current, goal);

    assertEquals(expectedResult, result.get());
  }

  @Test
  void testPlanNoCollisionsHigh() {
    SuperstructurePosition current = new SuperstructurePosition(67, 85);
    SuperstructurePosition goal = new SuperstructurePosition(65, 180);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(Optional.empty(), result);
  }
  @Test
  void testSkip345() {
    SuperstructurePosition current = new SuperstructurePosition(0, 45);
    SuperstructurePosition goal = new SuperstructurePosition(65, 45);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(Optional.empty(), result);
  }
}
