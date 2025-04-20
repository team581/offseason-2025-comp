package frc.robot.util.trailblazer.followers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.MathHelpers;
import frc.robot.util.kinematics.PolarChassisSpeeds;
import frc.robot.util.trailblazer.constraints.AutoConstraintOptions;

public class ProfiledPidPathFollower implements PathFollower {
  private final ProfiledPIDController velocityController;
  private final ProfiledPIDController rotationController;

  public ProfiledPidPathFollower(
      PIDController velocityController, PIDController rotationController) {
    this.velocityController =
        new ProfiledPIDController(
            velocityController.getP(),
            velocityController.getI(),
            velocityController.getD(),
            new AutoConstraintOptions().linearConstraints());
    this.rotationController =
        new ProfiledPIDController(
            rotationController.getP(),
            rotationController.getI(),
            rotationController.getD(),
            new AutoConstraintOptions().angularConstraints());

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public PolarChassisSpeeds calculateSpeeds(
      Pose2d currentPose, Pose2d targetPose, AutoConstraintOptions constraints) {
    return new PolarChassisSpeeds(
        Math.abs(
            velocityController.calculate(
                currentPose.getTranslation().getDistance(targetPose.getTranslation()),
                new TrapezoidProfile.State(0, 0),
                constraints.linearConstraints())),
        MathHelpers.getDriveDirection(targetPose, currentPose),
        rotationController.calculate(
            currentPose.getRotation().getRadians(),
            new TrapezoidProfile.State(targetPose.getRotation().getRadians(), 0),
            constraints.angularConstraints()));
  }
}
