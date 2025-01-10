package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;

import dev.doglog.DogLog;
import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
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

  public VisionSubsystem(ImuSubsystem imu, Limelight topLimelight, Limelight bottomLimelight, Limelight backLimelight) {
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
    if (topLimelight.getState() == LimelightState.TAGS) {
      var maybeResult = topLimelight.getInterpolatedVisionResult();
      if (maybeResult.isPresent()) {
        interpolatedVisionResult.add(maybeResult.get());
      }
    }

    if (bottomLimelight.getState() == LimelightState.TAGS) {
      var maybeResult = bottomLimelight.getInterpolatedVisionResult();
      if (maybeResult.isPresent()) {
        interpolatedVisionResult.add(maybeResult.get());
      }
    }
    if (backLimelight.getState() == LimelightState.TAGS) {
      var maybeResult = backLimelight.getInterpolatedVisionResult();
      if (maybeResult.isPresent()) {
        interpolatedVisionResult.add(maybeResult.get());
      }
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
    DogLog.log("Vision/AngularVelocity", angularVelocity);
    DogLog.log("Vision/Pitch", pitch);
    DogLog.log("Vision/visionIsEmpty", getInterpolatedVisionResult().isEmpty());

    DogLog.log("Vision/CombinedVisionState", getVisionState());
    DogLog.log("Vision/Left/VisionState", topLimelight.getCameraStatus());
    DogLog.log("Vision/Right/VisionState", bottomLimelight.getCameraStatus());
    DogLog.log("Vision/Back/VisionState", backLimelight.getCameraStatus());

  }

  public CameraStatus getVisionState() {
    var topStatus = topLimelight.getCameraStatus();
    var bottomStatus = bottomLimelight.getCameraStatus();
    var backStatus = backLimelight.getCameraStatus();

    if (topStatus == CameraStatus.OFFLINE && bottomStatus == CameraStatus.OFFLINE && backStatus == CameraStatus.OFFLINE) {
      return CameraStatus.OFFLINE;
    }

    if (topStatus == CameraStatus.GOOD || bottomStatus == CameraStatus.GOOD || backStatus == CameraStatus.GOOD) {
      return CameraStatus.GOOD;
    }

    return CameraStatus.NO_TARGETS;
  }

  /** Same as the regular vision state but returns OFFLINE if any camera is offline. */
  public CameraStatus getPessemisticVisionState() {
    var topStatus = topLimelight.getCameraStatus();
    var bottomStatus = bottomLimelight.getCameraStatus();
    var backStatus = backLimelight.getCameraStatus();

    if (topStatus == CameraStatus.OFFLINE || bottomStatus == CameraStatus.OFFLINE || backStatus == CameraStatus.OFFLINE) {
      return CameraStatus.OFFLINE;
    }

    if (topStatus == CameraStatus.GOOD || bottomStatus == CameraStatus.GOOD || backStatus == CameraStatus.GOOD) {
      return CameraStatus.GOOD;
    }

    return CameraStatus.NO_TARGETS;
  }
}
