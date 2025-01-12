package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayList;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {
  @Test
  void collidesTest() {
    var currentPose = new Pose2d(-1, 0, new Rotation2d());
    var goalPose = new Pose2d(1, 4, new Rotation2d());

    var result = CollisionAvoidanceUtils.collides(currentPose, goalPose);
    var expected = true;
    assertEquals(expected, result);
  }

  @Test
  void getGoalPointTest() {
    var currentPose = new Pose2d(-1, 0, new Rotation2d());
    var safePoint1 = new Pose2d(2, 0, new Rotation2d());
    var safePoint2 = new Pose2d(1.5, 1, new Rotation2d());
    var safePoint3 = new Pose2d(1.5, 2, new Rotation2d());
    ArrayList<Pose2d> possibleGoalPoints = new ArrayList<Pose2d>();
    possibleGoalPoints.set(0, CollisionAvoidanceUtils.angleHeightToPose(45.0, 5.0));
    possibleGoalPoints.set(1, safePoint1);
    possibleGoalPoints.set(2, safePoint2);
    possibleGoalPoints.set(3, safePoint3);
    var result = CollisionAvoidanceUtils.getGoalPoint(possibleGoalPoints, currentPose);
    var expected = new Pose2d(1.5, 1, new Rotation2d());
    assertEquals(expected, result);
  }

  @Test
  void testPlanNoCollisionsLow() {
    SuperstructurePosition current = new SuperstructurePosition(0, 150);
    SuperstructurePosition goal = new SuperstructurePosition(1, -15);
    SuperstructurePosition expectedResult = new SuperstructurePosition(1, -15);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(expectedResult, result);
  }

  @Test
  void testPlanNoCollisionsRight() {
    SuperstructurePosition current = new SuperstructurePosition(0, 0);
    SuperstructurePosition goal = new SuperstructurePosition(65, 80);
    SuperstructurePosition expectedResult = new SuperstructurePosition(65, 80);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(expectedResult, result);
  }

  @Test
  void testPlanNoCollisionsHigh() {
    SuperstructurePosition current = new SuperstructurePosition(65, 10);
    SuperstructurePosition goal = new SuperstructurePosition(68, 135);
    SuperstructurePosition expectedResult = new SuperstructurePosition(68, 135);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(expectedResult, result);
  }

  @Test
  void testPlanCollisionLowToMid() {
    SuperstructurePosition current = new SuperstructurePosition(0, 135);
    SuperstructurePosition goal = new SuperstructurePosition(20, 80);
    SuperstructurePosition expectedResult = new SuperstructurePosition(2, 75);
    var result = CollisionAvoidance.plan(current, goal);
    assertEquals(expectedResult, result);
  }
}
