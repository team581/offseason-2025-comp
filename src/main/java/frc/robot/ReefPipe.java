package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.fms.FmsSubsystem;

public enum ReefPipe {
  PIPE_A(new Pose2d(), new Pose2d()),
  PIPE_B(new Pose2d(), new Pose2d()),
  PIPE_C(new Pose2d(), new Pose2d()),
  PIPE_D(new Pose2d(), new Pose2d()),
  PIPE_E(new Pose2d(), new Pose2d()),
  PIPE_F(new Pose2d(), new Pose2d()),
  PIPE_G(new Pose2d(), new Pose2d()),
  PIPE_H(new Pose2d(), new Pose2d()),
  PIPE_I(new Pose2d(), new Pose2d()),
  PIPE_J(new Pose2d(), new Pose2d()),
  PIPE_K(new Pose2d(), new Pose2d()),
  PIPE_L(new Pose2d(), new Pose2d());

  public final Pose2d redPose;
  public final Pose2d bluePose;

  ReefPipe(Pose2d redPose, Pose2d bluePose) {
    this.redPose = redPose;
    this.bluePose = bluePose;
  }
  public Pose2d getPose(){
    if (FmsSubsystem.isRedAlliance()){
      return redPose;

    }
    else{
      return bluePose;
    }
  }
}
