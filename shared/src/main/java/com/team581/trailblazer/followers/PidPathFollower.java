package com.team581.trailblazer.followers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class PidPathFollower implements PathFollower {
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;

  private static final double ROTATION_FEED_FORWARD = 0.01;

  public PidPathFollower(
      PIDController xController, PIDController yController, PIDController thetaController) {
    this.xController = xController;
    this.yController = yController;
    this.thetaController = thetaController;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(0.01);
  }

  @Override
  public ChassisSpeeds calculateSpeeds(Pose2d currentPose, Pose2d targetPose) {
    double rotationSpeed =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    if (!MathUtil.isNear(
        targetPose.getRotation().getDegrees(),
        currentPose.getRotation().getDegrees(),
        1.0,
        -180,
        180)) {
      rotationSpeed +=
          Math.copySign(Units.rotationsToRadians(ROTATION_FEED_FORWARD), rotationSpeed);
    }

    return new ChassisSpeeds(
        xController.calculate(currentPose.getX(), targetPose.getX()),
        yController.calculate(currentPose.getY(), targetPose.getY()),
        rotationSpeed);
  }
}
