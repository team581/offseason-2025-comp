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
  private final Limelight elevatorPurpleLimelight;
  private final Limelight frontCoralLimelight;
  private final Limelight backTagLimelight;
  private final List<TagResult> interpolatedVisionResult = new ArrayList<>();
  private double robotHeading;
  private double pitch;
  private double angularVelocity;
  private double pitchRate;
  private double roll;
  private double rollRate;

  public VisionSubsystem(
      ImuSubsystem imu,
      Limelight elevatorPurpleLimelight,
      Limelight frontCoralLimelight,
      Limelight backTagLimelight) {
    super(SubsystemPriority.VISION, VisionState.DEFAULT_STATE);
    this.imu = imu;
    this.elevatorPurpleLimelight = elevatorPurpleLimelight;
    this.frontCoralLimelight = frontCoralLimelight;
    this.backTagLimelight = backTagLimelight;
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
    var maybeTopResult = elevatorPurpleLimelight.getInterpolatedTagResult();
    var maybeBottomResult = frontCoralLimelight.getInterpolatedTagResult();
    var maybeBackResult = backTagLimelight.getInterpolatedTagResult();

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
    elevatorPurpleLimelight.sendImuData(
        robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    frontCoralLimelight.sendImuData(
        robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    backTagLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);

    DogLog.log("Vision/visionIsEmpty", getInterpolatedVisionResult().isEmpty());
    DogLog.log("Vision/CombinedVisionState", getVisionState());
    DogLog.log("Vision/Left/VisionState", elevatorPurpleLimelight.getCameraHealth());
    DogLog.log("Vision/Right/VisionState", frontCoralLimelight.getCameraHealth());
    DogLog.log("Vision/Back/VisionState", backTagLimelight.getCameraHealth());
  }

  public boolean isAnyTagLimelightOnline() {
    if ((frontCoralLimelight.getState() == LimelightState.TAGS
            || frontCoralLimelight.getState() == LimelightState.REEF_TAGS)
        && frontCoralLimelight.getCameraHealth() == CameraHealth.OFFLINE) {
      return false;
    }
    if ((backTagLimelight.getState() == LimelightState.TAGS
            || backTagLimelight.getState() == LimelightState.REEF_TAGS)
        && backTagLimelight.getCameraHealth() == CameraHealth.OFFLINE) {
      return false;
    }

    return true;
  }

  public CameraHealth getVisionState() {
    var topStatus = elevatorPurpleLimelight.getCameraHealth();
    var bottomStatus = frontCoralLimelight.getCameraHealth();
    var backStatus = backTagLimelight.getCameraHealth();

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
    var topStatus = elevatorPurpleLimelight.getCameraHealth();
    var bottomStatus = frontCoralLimelight.getCameraHealth();
    var backStatus = backTagLimelight.getCameraHealth();

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
