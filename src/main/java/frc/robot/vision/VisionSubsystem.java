package frc.robot.vision;

import dev.doglog.DogLog;
import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import frc.robot.vision.results.TagResult;
import java.util.ArrayList;
import java.util.List;

public class VisionSubsystem extends StateMachine<VisionState> {
  private final ImuSubsystem imu;
  private final Limelight topPurpleLimelight;
  private final Limelight bottomCoralLimelight;
  private final Limelight backwardsTagLimelight;
  private final List<TagResult> interpolatedVisionResult = new ArrayList<>();
  private double robotHeading;
  private double pitch;
  private double angularVelocity;
  private double pitchRate;
  private double roll;
  private double rollRate;

  public VisionSubsystem(
      ImuSubsystem imu,
      Limelight topPurpleLimelight,
      Limelight bottomCoralLimelight,
      Limelight backwardsTagLimelight) {
    super(SubsystemPriority.VISION, VisionState.DEFAULT_STATE);
    this.imu = imu;
    this.topPurpleLimelight = topPurpleLimelight;
    this.bottomCoralLimelight = bottomCoralLimelight;
    this.backwardsTagLimelight = backwardsTagLimelight;
  }

  @Override
  protected void collectInputs() {
    robotHeading = imu.getRobotHeading();
    DogLog.log("Vision/RobotHeading", imu.getRobotHeading());
    angularVelocity = imu.getRobotAngularVelocity();
    pitch = imu.getPitch();
    pitchRate = imu.getPitchRate();
    roll = imu.getRoll();
    rollRate = imu.getRollRate();

    interpolatedVisionResult.clear();
    var maybeTopResult = topPurpleLimelight.getInterpolatedTagResult();
    var maybeBottomResult = bottomCoralLimelight.getInterpolatedTagResult();
    var maybeBackResult = backwardsTagLimelight.getInterpolatedTagResult();

    if (maybeTopResult.isPresent()) {
      interpolatedVisionResult.add(maybeTopResult.get());
    }

    if (maybeBottomResult.isPresent()) {
      interpolatedVisionResult.add(maybeBottomResult.get());
    }

    if (maybeBackResult.isPresent()) {
      interpolatedVisionResult.add(maybeBackResult.get());
    }
  }

  public List<TagResult> getInterpolatedVisionResult() {
    return interpolatedVisionResult;
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    topPurpleLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    bottomCoralLimelight.sendImuData(
        robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    backwardsTagLimelight.sendImuData(
        robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);

    DogLog.log("Vision/visionIsEmpty", getInterpolatedVisionResult().isEmpty());
    DogLog.log("Vision/CombinedVisionState", getVisionState());
    DogLog.log("Vision/Left/VisionState", topPurpleLimelight.getCameraHealth());
    DogLog.log("Vision/Right/VisionState", bottomCoralLimelight.getCameraHealth());
    DogLog.log("Vision/Back/VisionState", backwardsTagLimelight.getCameraHealth());
  }

  public boolean isAnyTagLimelightOnline() {
    if ((bottomCoralLimelight.getState() == LimelightState.TAGS
            || bottomCoralLimelight.getState() == LimelightState.REEF_TAGS)
        && bottomCoralLimelight.getCameraHealth() == CameraHealth.OFFLINE) {
      return false;
    }
    if ((backwardsTagLimelight.getState() == LimelightState.TAGS
            || backwardsTagLimelight.getState() == LimelightState.REEF_TAGS)
        && backwardsTagLimelight.getCameraHealth() == CameraHealth.OFFLINE) {
      return false;
    }

    return true;
  }

  public CameraHealth getVisionState() {
    var topStatus = topPurpleLimelight.getCameraHealth();
    var bottomStatus = bottomCoralLimelight.getCameraHealth();
    var backStatus = backwardsTagLimelight.getCameraHealth();

    if (topStatus == CameraHealth.OFFLINE
        && bottomStatus == CameraHealth.OFFLINE
        && backStatus == CameraHealth.OFFLINE) {
      return CameraHealth.OFFLINE;
    }

    if (topStatus == CameraHealth.GOOD
        || bottomStatus == CameraHealth.GOOD
        || backStatus == CameraHealth.GOOD) {
      return CameraHealth.GOOD;
    }

    return CameraHealth.NO_TARGETS;
  }

  /** Same as the regular vision state but returns OFFLINE if any camera is offline. */
  public CameraHealth getPessemisticVisionState() {
    var topStatus = topPurpleLimelight.getCameraHealth();
    var bottomStatus = bottomCoralLimelight.getCameraHealth();
    var backStatus = backwardsTagLimelight.getCameraHealth();

    if (topStatus == CameraHealth.OFFLINE
        || bottomStatus == CameraHealth.OFFLINE
        || backStatus == CameraHealth.OFFLINE) {
      return CameraHealth.OFFLINE;
    }

    if (topStatus == CameraHealth.GOOD
        || bottomStatus == CameraHealth.GOOD
        || backStatus == CameraHealth.GOOD) {
      return CameraHealth.GOOD;
    }

    return CameraHealth.NO_TARGETS;
  }
}
