package frc.robot.util.trailblazer.followers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.MathHelpers;
import frc.robot.util.kinematics.PolarChassisSpeeds;
import frc.robot.util.trailblazer.constraints.AutoConstraintOptions;

public class PidPathFollower implements PathFollower {
  private final PIDController velocityController;
  private final PIDController rotationController;

  public PidPathFollower(PIDController velocityController, PIDController rotationController) {
    this.velocityController = velocityController;
    this.rotationController = rotationController;

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public PolarChassisSpeeds calculateSpeeds(
      Pose2d currentPose, Pose2d targetPose, AutoConstraintOptions constraints) {
    return new PolarChassisSpeeds(
        Math.abs(
            velocityController.calculate(
                currentPose.getTranslation().getDistance(targetPose.getTranslation()), 0)),
        MathHelpers.getDriveDirection(targetPose, currentPose),
        rotationController.calculate(currentPose.getRotation().getRadians(), 0));
  }
}
