package frc.robot.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.auto_align.ReefPipe;
import frc.robot.config.FeatureFlags;
import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import frc.robot.vision.results.GamePieceResult;
import frc.robot.vision.results.TagResult;
import java.util.ArrayDeque;
import java.util.Collection;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Queue;

public class VisionSubsystem extends StateMachine<VisionState> {
  private static final double REEF_CLOSEUP_DISTANCE = 0.7;
  private static final Debouncer HAS_SEEN_TAG_DISABLED_DEBOUNCE =
      new Debouncer(0.5, DebounceType.kFalling);
  private final ImuSubsystem imu;
  private final Limelight leftBackLimelight;
  private final Limelight leftFrontLimelight;
  private final Limelight rightLimelight;
  private final Limelight gamePieceDetectionLimelight;

  private final Queue<TagResult> tagResult = new ArrayDeque<>(4);
  private static final double LAST_SEEN_TAG_DISABLED_TIMESTAMP = 0.0;
  private double robotHeading;
  private double pitch;
  private double angularVelocity;
  private double pitchRate;
  private double roll;
  private double rollRate;
  private ReefPipe reefPipe;

  private boolean hasSeenTag = false;

  public VisionSubsystem(
      ImuSubsystem imu,
      Limelight leftBackLimelight,
      Limelight leftFrontLimelight,
      Limelight rightLimelight,
      Limelight gamePieceDetectionLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.leftBackLimelight = leftBackLimelight;
    this.leftFrontLimelight = leftFrontLimelight;
    this.rightLimelight = rightLimelight;
    this.gamePieceDetectionLimelight = gamePieceDetectionLimelight;
  }

  @Override
  protected void collectInputs() {
    robotHeading = imu.getRobotHeading();
    angularVelocity = imu.getRobotAngularVelocity();
    pitch = imu.getPitch();
    pitchRate = imu.getPitchRate();
    roll = imu.getRoll();
    rollRate = imu.getRollRate();

    if (getState() == VisionState.CLOSEST_REEF_TAG_CLOSEUP) {
      switch (reefPipe) {
        case PIPE_A, PIPE_C, PIPE_E, PIPE_G, PIPE_I, PIPE_K -> {
          if (leftFrontLimelight.getCameraHealth() != CameraHealth.OFFLINE) {
            leftBackLimelight.setState(LimelightState.OFF);
            leftFrontLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
          }
        }
        case PIPE_B, PIPE_D, PIPE_F, PIPE_H, PIPE_J, PIPE_L -> {
          if (leftBackLimelight.getCameraHealth() != CameraHealth.OFFLINE) {
            leftFrontLimelight.setState(LimelightState.OFF);
            leftBackLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
          }
        }
      }
    }

    tagResult.clear();
    var maybeLeftBackResult = leftBackLimelight.getTagResult();
    var maybeleftFrontResult = leftFrontLimelight.getTagResult();
    var maybeFrontResult = rightLimelight.getTagResult();
    var maybeGamePieceResult = gamePieceDetectionLimelight.getTagResult();

    if (maybeLeftBackResult.isPresent()) {
      tagResult.add(maybeLeftBackResult.orElseThrow());
    }

    if (maybeleftFrontResult.isPresent()) {
      tagResult.add(maybeleftFrontResult.orElseThrow());
    }

    if (maybeFrontResult.isPresent()) {
      tagResult.add(maybeFrontResult.orElseThrow());
    }

    if (maybeGamePieceResult.isPresent()) {
      tagResult.add(maybeGamePieceResult.get());
    }

    if (!hasSeenTag) {
      hasSeenTag = !tagResult.isEmpty();
    }
  }

  public Collection<TagResult> getTagResult() {
    return tagResult;
  }

  public boolean hasSeenTagRecentlyDisabled() {
    return HAS_SEEN_TAG_DISABLED_DEBOUNCE.calculate(!getTagResult().isEmpty());
  }

  public boolean hasSeenTag() {
    return hasSeenTag;
  }

  public void setState(VisionState state) {
    setStateFromRequest(state);
  }

  @Override
  protected void afterTransition(VisionState newState) {
    switch (newState) {
      case TAGS -> {
        leftBackLimelight.setState(LimelightState.TAGS);
        leftFrontLimelight.setState(LimelightState.TAGS);
        rightLimelight.setState(LimelightState.TAGS);
        gamePieceDetectionLimelight.setState(LimelightState.CORAL);
      }
      case CLOSEST_REEF_TAG -> {
        leftBackLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
        leftFrontLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
        rightLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
        gamePieceDetectionLimelight.setState(LimelightState.CORAL);
      }
      case CLOSEST_REEF_TAG_CLOSEUP -> {
        leftBackLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
        leftFrontLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
        rightLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
        gamePieceDetectionLimelight.setState(LimelightState.CORAL);
      }
      case CORAL_DETECTION -> {
        leftBackLimelight.setState(LimelightState.TAGS);
        leftFrontLimelight.setState(LimelightState.TAGS);
        rightLimelight.setState(LimelightState.TAGS);
        gamePieceDetectionLimelight.setState(LimelightState.CORAL);
      }
      case ALGAE_DETECTION -> {
        leftBackLimelight.setState(LimelightState.TAGS);
        leftFrontLimelight.setState(LimelightState.TAGS);
        rightLimelight.setState(LimelightState.ALGAE);
        gamePieceDetectionLimelight.setState(LimelightState.ALGAE);
      }
    }
  }

  public Optional<GamePieceResult> getLollipopVisionResult() {
    return rightLimelight.getAlgaeResult();
  }

  public OptionalDouble getHandoffOffsetResult() {
    return gamePieceDetectionLimelight.coralHandoff();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    leftBackLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    leftFrontLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    rightLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    gamePieceDetectionLimelight.sendImuData(
        robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    if (FeatureFlags.CAMERA_POSITION_CALIBRATION.getAsBoolean()) {
      setStateFromRequest(VisionState.TAGS);
      leftBackLimelight.logCameraPositionCalibrationValues();
      leftFrontLimelight.logCameraPositionCalibrationValues();
      rightLimelight.logCameraPositionCalibrationValues();
      gamePieceDetectionLimelight.logCameraPositionCalibrationValues();
    }
  }

  public void setClosestScoringReefAndPipe(int tagID, ReefPipe currentScoringPipe) {
    reefPipe = currentScoringPipe;
    leftFrontLimelight.setClosestScoringReefTag(tagID);
    rightLimelight.setClosestScoringReefTag(tagID);
    leftBackLimelight.setClosestScoringReefTag(tagID);
    gamePieceDetectionLimelight.setClosestScoringReefTag(tagID);
  }

  public boolean isAnyCameraOffline() {

    if (RobotBase.isSimulation()) {
      return false;
    }
    return leftBackLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || leftFrontLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || rightLimelight.getCameraHealth() == CameraHealth.OFFLINE;
    // || gamePieceDetectionLimelight.getCameraHealth() == CameraHealth.OFFLINE;

  }

  public boolean isAnyLeftScoringTagLimelightOnline() {
    if ((leftBackLimelight.getState() == LimelightState.TAGS
            || leftBackLimelight.getState() == LimelightState.CLOSEST_REEF_TAG
            || leftBackLimelight.getState() == LimelightState.CLOSEST_REEF_TAG_CLOSEUP)
        && (leftBackLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || leftBackLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }
    if ((leftFrontLimelight.getState() == LimelightState.TAGS
            || leftFrontLimelight.getState() == LimelightState.CLOSEST_REEF_TAG
            || leftFrontLimelight.getState() == LimelightState.CLOSEST_REEF_TAG_CLOSEUP)
        && (leftFrontLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || leftFrontLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }

    if (RobotBase.isSimulation()) {
      return true;
    }
    return false;
  }

  public boolean isAnyRightScoringTagLimelightOnline() {
    if ((rightLimelight.getState() == LimelightState.TAGS
            || rightLimelight.getState() == LimelightState.CLOSEST_REEF_TAG
            || rightLimelight.getState() == LimelightState.CLOSEST_REEF_TAG_CLOSEUP)
        && (rightLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || rightLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }

    if (RobotBase.isSimulation()) {
      return true;
    }
    return false;
  }

  public boolean isAnyTagLimelightOnline() {
    if ((leftBackLimelight.getState() == LimelightState.TAGS
            || leftBackLimelight.getState() == LimelightState.CLOSEST_REEF_TAG
            || leftBackLimelight.getState() == LimelightState.CLOSEST_REEF_TAG_CLOSEUP)
        && (leftBackLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || leftBackLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }
    if ((rightLimelight.getState() == LimelightState.TAGS
            || rightLimelight.getState() == LimelightState.CLOSEST_REEF_TAG
            || rightLimelight.getState() == LimelightState.CLOSEST_REEF_TAG_CLOSEUP)
        && (rightLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || rightLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }
    if ((leftFrontLimelight.getState() == LimelightState.TAGS
            || leftFrontLimelight.getState() == LimelightState.CLOSEST_REEF_TAG
            || leftFrontLimelight.getState() == LimelightState.CLOSEST_REEF_TAG_CLOSEUP)
        && (leftFrontLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || leftFrontLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }

    if (RobotBase.isSimulation()) {
      return true;
    }

    return false;
  }

  public void updateDistanceFromReef(double distanceFromReef) {
    DogLog.log("Vision/DistanceFromReef", distanceFromReef);
    if (getState() == VisionState.CLOSEST_REEF_TAG) {
      if (distanceFromReef < REEF_CLOSEUP_DISTANCE) {
        setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
      }
    }
  }
}
