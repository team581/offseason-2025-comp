package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.robot_manager.SuperstructurePosition;
import frc.robot.util.MathHelpers;
import java.util.Optional;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {
  @Test
  void collidesTest() {
    var currentPose = CollisionAvoidance.angleHeightToPose(135, 0);
    var goalPose = CollisionAvoidance.angleHeightToPose(90, 0);
    var result = CollisionAvoidance.collides(currentPose, goalPose);
    var expected = false;
    assertEquals(expected, result);
  }

  @Test
  void collidesTestFromSuperStructure() {
    var currentPose = CollisionAvoidance.angleHeightToPose(80, 65);
    var goalPose = CollisionAvoidance.angleHeightToPose(0, 0);

    var result = CollisionAvoidance.collides(currentPose, goalPose);
    var expected = false;
    assertEquals(expected, result);
  }

  @Test
  void collidesTestFromSuperStructure5() {
    var currentPose = CollisionAvoidance.angleHeightToPose(80, 20);
    var goalPose = CollisionAvoidance.angleHeightToPose(135, 0);

    var result = CollisionAvoidance.collides(currentPose, goalPose);
    var expected = true;
    assertEquals(expected, result);
  }

  @Test
  void collidesTestFromSuperStructure2() {
    var currentPose = CollisionAvoidance.angleHeightToPose(10, 90);
    var goalPose = CollisionAvoidance.angleHeightToPose(0, 90);

    var result = CollisionAvoidance.collides(currentPose, goalPose);
    var expected = false;
    assertEquals(expected, result);
  }

  @Test
  void collidesTestFromSuperStructure3() {
    var currentPose = CollisionAvoidance.angleHeightToPose(10, 90);
    var goalPose = CollisionAvoidance.angleHeightToPose(0, 0);

    var result = CollisionAvoidance.collides(currentPose, goalPose);
    var expected = false;
    assertEquals(expected, result);
  }

  @Test
  void angleHeightToPoseTest() {
    // 2.0, 135.0
    var wristAngle = 135.0;
    var elevatorHeight = 6.4436508139;

    var result = CollisionAvoidance.angleHeightToPose(wristAngle, elevatorHeight);
    var expected = new Translation2d(-15.56, 22.00);
    var roundedResult =
        new Translation2d(
            MathHelpers.roundTo(result.getX(), 0.01), MathHelpers.roundTo(result.getY(), 0.01));
    assertEquals(expected, roundedResult);
  }

  @Test
  void distancefromPosesTest() {
    var currentPose = new Translation2d(4.0, 0.0);
    var goalPose = new Translation2d(4.0, 0.0);

    var result = CollisionAvoidance.distancefromPoses(currentPose, goalPose);
    var expected = 0;
    assertEquals(expected, result);
  }

  @Test
  void testPlanNoCollisionsLow() {
    SuperstructurePosition current = new SuperstructurePosition(0, 150);
    SuperstructurePosition goal = new SuperstructurePosition(1, -15);
    SuperstructurePosition expectedResult = new SuperstructurePosition(1, -15);
    var result = CollisionAvoidance.plan(current, goal);

    assertEquals(Optional.empty(), result);
  }

  @Test
  void testPlanNoCollisionsRight() {
    SuperstructurePosition current = new SuperstructurePosition(0, 0);
    SuperstructurePosition goal = new SuperstructurePosition(65, 80);
    SuperstructurePosition expectedResult = new SuperstructurePosition(65, 80);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(Optional.empty(), result);
  }

  @Test
  void testPlanNoCollisionsHigh() {
    SuperstructurePosition current = new SuperstructurePosition(90, 10);
    SuperstructurePosition goal = new SuperstructurePosition(65, 85);
    var result = CollisionAvoidance.plan(current, goal);
    var expected = new SuperstructurePosition(90, 0);
    assertEquals(expected, result.get()); //
  }

  @Test
  void testPlanCollisionLowToMid() {
    SuperstructurePosition current = new SuperstructurePosition(0, 135);
    SuperstructurePosition goal = new SuperstructurePosition(20, 80);
    SuperstructurePosition expectedResult = new SuperstructurePosition(0.0, 0.0);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(expectedResult, result.get());
  }

  @Test
  void testPlanCollisionLowToHigh() {
    SuperstructurePosition current = new SuperstructurePosition(0, 135);
    SuperstructurePosition goal = new SuperstructurePosition(88, 90);
    SuperstructurePosition expectedResult = new SuperstructurePosition(0.0, 0.0);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(expectedResult, result.get());
  }

  @Test
  void testPlanNoCollisionSuperClose() {
    SuperstructurePosition current = new SuperstructurePosition(0, 135);
    SuperstructurePosition goal = new SuperstructurePosition(0, 90);
    SuperstructurePosition expectedResult = new SuperstructurePosition(0.0, 90.0);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(Optional.empty(), result);
  }
}
