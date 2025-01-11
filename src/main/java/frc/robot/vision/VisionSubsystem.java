package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import dev.doglog.DogLog;
import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.results.TagResult;

public class VisionSubsystem extends StateMachine<VisionState> {
  private final ImuSubsystem imu;
  private final Limelight topLimelight;
  private final Limelight bottomLimelight;
  private final Limelight backLimelight;
  private final List<TagResult> interpolatedVisionResult = new ArrayList<>();
  private double robotHeading;
  private double pitch;
  private double angularVelocity;
  private double pitchRate;
  private double roll;
  private double rollRate;

  public VisionSubsystem(
      ImuSubsystem imu,
      Limelight topLimelight,
      Limelight bottomLimelight,
      Limelight backLimelight) {
    super(SubsystemPriority.VISION, VisionState.DEFAULT_STATE);
    this.imu = imu;
    this.topLimelight = topLimelight;
    this.bottomLimelight = bottomLimelight;
    this.backLimelight = backLimelight;
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
    var maybeTopResult = topLimelight.getInterpolatedTagResult();
    var maybeBottomResult = bottomLimelight.getInterpolatedTagResult();
    var maybeBackResult = backLimelight.getInterpolatedTagResult();

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
    topLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    bottomLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    backLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);

    DogLog.log("Vision/visionIsEmpty", getInterpolatedVisionResult().isEmpty());
    DogLog.log("Vision/CombinedVisionState", getVisionState());
    DogLog.log("Vision/Left/VisionState", topLimelight.getCameraHealth());
    DogLog.log("Vision/Right/VisionState", bottomLimelight.getCameraHealth());
    DogLog.log("Vision/Back/VisionState", backLimelight.getCameraHealth());
  }

  public CameraHealth getVisionState() {
    var topStatus = topLimelight.getCameraHealth();
    var bottomStatus = bottomLimelight.getCameraHealth();
    var backStatus = backLimelight.getCameraHealth();

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
    var topStatus = topLimelight.getCameraHealth();
    var bottomStatus = bottomLimelight.getCameraHealth();
    var backStatus = backLimelight.getCameraHealth();

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
