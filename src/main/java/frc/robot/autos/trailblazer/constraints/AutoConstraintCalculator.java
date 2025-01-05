package frc.robot.autos.trailblazer.constraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoConstraintCalculator {
  public static Pose2d constrainTargetPose(Pose2d inputPose, AutoConstraintOptions options) {
    if (options.collisionAvoidance()) {
      // TODO: Implement collision avoidance
    }

    return inputPose;
  }

  public static ChassisSpeeds constrainVelocityGoal(
      ChassisSpeeds inputSpeeds, AutoConstraintOptions options) {
    if (options.maxLinearVelocity() != 0) {
      // TODO: Implement linear velocity constraint
    }

    if (options.maxAngularVelocity() != 0) {
      // TODO: Implement angular velocity constraint
    }

    if (options.maxLinearAcceleration() != 0) {
      // TODO: Implement linear acceleration constraint
    }

    if (options.maxAngularAcceleration() != 0) {
      // TODO: Implement angular acceleration constraint
    }

    return inputSpeeds;
  }

  private AutoConstraintCalculator() {}
}
