package frc.robot.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auto_align.ReefPipe;
import frc.robot.config.FeatureFlags;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import frc.robot.vision.results.GamePieceResult;
import frc.robot.vision.results.TagResult;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionSubsystem extends StateMachine<VisionState> {
  private static final double REEF_CLOSEUP_DISTANCE = 1.0;
  private final ImuSubsystem imu;
  private final Limelight leftBackLimelight;
  private final Limelight leftFrontLimelight;
  private final Limelight rightLimelight;

  private final List<TagResult> tagResult = new ArrayList<>();
  private double robotHeading;
  private double pitch;
  private double angularVelocity;
  private double pitchRate;
  private double roll;
  private double rollRate;
  private ReefPipe reefPipe;

  public VisionSubsystem(
      ImuSubsystem imu,
      Limelight leftBackLimelight,
      Limelight leftFrontLimelight,
      Limelight rightLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.leftBackLimelight = leftBackLimelight;
    this.leftFrontLimelight = leftFrontLimelight;
    this.rightLimelight = rightLimelight;
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
            leftFrontLimelight.setState(LimelightState.OFF);
            leftBackLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
          }
        }
        case PIPE_B, PIPE_D, PIPE_F, PIPE_H, PIPE_J, PIPE_L -> {
          if (leftBackLimelight.getCameraHealth() != CameraHealth.OFFLINE) {
            leftBackLimelight.setState(LimelightState.OFF);
            leftFrontLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
          }
        }
      }
    }

    tagResult.clear();
    var maybeLeftBackResult = leftBackLimelight.getTagResult();
    var maybeleftFrontResult = leftFrontLimelight.getTagResult();
    var maybeFrontResult = rightLimelight.getTagResult();

    if (maybeLeftBackResult.isPresent()) {
      tagResult.add(maybeLeftBackResult.orElseThrow());
    }

    if (maybeleftFrontResult.isPresent()) {
      tagResult.add(maybeleftFrontResult.orElseThrow());
    }

    if (maybeFrontResult.isPresent()) {
      tagResult.add(maybeFrontResult.orElseThrow());
    }
  }

  public List<TagResult> getTagResult() {
    return tagResult;
  }

  public void setState(VisionState state) {
    setStateFromRequest(state);
  }

  public void setCurrentScoringPipe(ReefPipe reefPipe) {
    this.reefPipe = reefPipe;
  }

  @Override
  protected void afterTransition(VisionState newState) {
    switch (newState) {
      case TAGS -> {
        leftBackLimelight.setState(LimelightState.TAGS);
        leftFrontLimelight.setState(LimelightState.TAGS);
        rightLimelight.setState(LimelightState.TAGS);
      }
      case CLOSEST_REEF_TAG -> {
        leftBackLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
        leftFrontLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
        rightLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
      }
      case CLOSEST_REEF_TAG_CLOSEUP -> {
        leftBackLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
        leftFrontLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
        rightLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
      }
      case STATION_TAGS -> {
        leftBackLimelight.setState(LimelightState.STATION_TAGS);
        leftFrontLimelight.setState(LimelightState.STATION_TAGS);
        rightLimelight.setState(LimelightState.STATION_TAGS);
      }
      case CORAL_DETECTION -> {
        leftBackLimelight.setState(LimelightState.TAGS);
        leftFrontLimelight.setState(LimelightState.TAGS);
        rightLimelight.setState(LimelightState.TAGS);
      }
      case ALGAE_DETECTION -> {
        leftBackLimelight.setState(LimelightState.TAGS);
        leftFrontLimelight.setState(LimelightState.TAGS);
        rightLimelight.setState(LimelightState.TAGS);
      }
    }
  }

  public Optional<GamePieceResult> getLollipopVisionResult() {
    return leftFrontLimelight.getAlgaeResult();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    leftBackLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    leftFrontLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    rightLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);

    if (FeatureFlags.CAMERA_POSITION_CALIBRATION.getAsBoolean()) {
      leftBackLimelight.logCameraPositionCalibrationValues();
      leftFrontLimelight.logCameraPositionCalibrationValues();
      rightLimelight.logCameraPositionCalibrationValues();
    }
  }

  public void setClosestScoringReefAndPipe(int tagID, ReefPipe currentScoringPipe) {
    reefPipe = currentScoringPipe;
    leftFrontLimelight.setClosestScoringReefTag(tagID);
    rightLimelight.setClosestScoringReefTag(tagID);
    leftBackLimelight.setClosestScoringReefTag(tagID);
  }

  public boolean isAnyCameraOffline() {
    return leftBackLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || leftFrontLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || rightLimelight.getCameraHealth() == CameraHealth.OFFLINE;
  }

  public boolean isAnyScoringTagLimelightOnline() {
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

  public Optional<Pose2d> getLollipopPose(LocalizationSubsystem localization) {
    // TODO: Update for new camera setup
    return Optional.empty();
    // var maybeAlgaeResult = frontCoralLimelight.getAlgaeResult();

    // if (maybeAlgaeResult.isEmpty()) {
    //   return Optional.empty();
    // }

    // var algaeResult = maybeAlgaeResult.orElseThrow();
    // var angleToCoral =
    //     GamePieceDetectionUtil.getFieldRelativeAngleToGamePiece(
    //         localization.getPose(algaeResult.timestamp()), algaeResult);

    // return Optional.of(
    //     new Pose2d(
    //         GamePieceDetectionUtil.calculateFieldRelativeLollipopTranslationFromCamera(
    //             localization.getPose(algaeResult.timestamp()), algaeResult),
    //         Rotation2d.fromDegrees(angleToCoral)));
  }
}
