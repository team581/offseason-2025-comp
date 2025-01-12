package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class CollisionAvoidanceTest {
  @Test
  void collidesTest() {
    var currentPose = new Pose2d(-1,0,new Rotation2d());
    var goalPose = new Pose2d(1,4, new Rotation2d());

    var result= CollisionAvoidanceUtils.collides(currentPose, goalPose);
    var expected = true;
    assertEquals(expected, result);
  }
  void getGoalPointTest(){
    var currentPose = new Pose2d(-1,0,new Rotation2d());
    var safePoint1 = new Pose2d(2,0,new Rotation2d());
    var safePoint2 = new Pose2d(1.5,1, new Rotation2d());
    var safePoint3 = new Pose2d(1.5,2,new Rotation2d());
    ArrayList<Pose2d> possibleGoalPoints = new ArrayList<Pose2d>();
possibleGoalPoints.set(0,CollisionAvoidanceUtils.angleHeightToPose(45.0,5.0));
          possibleGoalPoints.set(1,safePoint1);
          possibleGoalPoints.set(2,safePoint2);
          possibleGoalPoints.set(3,safePoint3);
    var result= CollisionAvoidanceUtils.getGoalPoint(possibleGoalPoints, currentPose);
    var expected =new Pose2d(1.5,1,new Rotation2d()) ;
    assertEquals(expected, result);
  }
}
