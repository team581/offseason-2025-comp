package frc.robot.vision;

import frc.robot.imu.ImuSubsystem;
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
  private final ImuSubsystem imu;
  private final Limelight frontCoralLimelight;
  private final Limelight backTagLimelight;
  private final Limelight frontRightLimelight;
  private final Limelight frontLeftLimelight;

  private final List<TagResult> tagResult = new ArrayList<>();
  private double robotHeading;
  private double pitch;
  private double angularVelocity;
  private double pitchRate;
  private double roll;
  private double rollRate;

  public VisionSubsystem(
      ImuSubsystem imu,
      Limelight frontCoralLimelight,
      Limelight backTagLimelight,
      Limelight frontRightLimelight,
      Limelight frontLeftLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.frontCoralLimelight = frontCoralLimelight;
    this.backTagLimelight = backTagLimelight;
    this.frontRightLimelight = frontRightLimelight;
    this.frontLeftLimelight = frontLeftLimelight;
  }

  @Override
  protected void collectInputs() {
    robotHeading = imu.getRobotHeading();
    angularVelocity = imu.getRobotAngularVelocity();
    pitch = imu.getPitch();
    pitchRate = imu.getPitchRate();
    roll = imu.getRoll();
    rollRate = imu.getRollRate();

    tagResult.clear();
    var maybeBottomResult = frontCoralLimelight.getTagResult();
    var maybeBackResult = backTagLimelight.getTagResult();
    var maybeFrontRightResult = frontRightLimelight.getTagResult();
    var maybeFrontLeftResult = frontLeftLimelight.getTagResult();

    if (maybeBottomResult.isPresent()) {
      tagResult.add(maybeBottomResult.orElseThrow());
    }

    if (maybeBackResult.isPresent()) {
      tagResult.add(maybeBackResult.orElseThrow());
    }

    if (maybeFrontRightResult.isPresent()) {
      tagResult.add(maybeFrontRightResult.orElseThrow());
    }

    if (maybeFrontLeftResult.isPresent()) {
      tagResult.add(maybeFrontLeftResult.orElseThrow());
    }
  }

  public List<TagResult> getTagResult() {
    return tagResult;
  }

  public void setState(VisionState state) {
    setStateFromRequest(state);
  }

  @Override
  protected void afterTransition(VisionState newState) {
    switch (newState) {
      case TAGS -> {
        frontCoralLimelight.setState(LimelightState.CORAL);
        backTagLimelight.setState(LimelightState.TAGS);
        frontRightLimelight.setState(LimelightState.TAGS);
        frontLeftLimelight.setState(LimelightState.TAGS);
      }
      case CLOSEST_REEF_TAG -> {
        frontCoralLimelight.setState(LimelightState.CORAL);
        backTagLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
        frontRightLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
        frontLeftLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
      }
      case STATION_TAGS -> {
        frontCoralLimelight.setState(LimelightState.CORAL);
        backTagLimelight.setState(LimelightState.STATION_TAGS);
        frontRightLimelight.setState(LimelightState.STATION_TAGS);
        frontLeftLimelight.setState(LimelightState.STATION_TAGS);
      }
      case CORAL_DETECTION -> {
        frontCoralLimelight.setState(LimelightState.CORAL);
        backTagLimelight.setState(LimelightState.TAGS);
        frontRightLimelight.setState(LimelightState.TAGS);
        frontLeftLimelight.setState(LimelightState.TAGS);
      }
      case ALGAE_DETECTION -> {
        frontCoralLimelight.setState(LimelightState.ALGAE);
        backTagLimelight.setState(LimelightState.TAGS);
        frontRightLimelight.setState(LimelightState.TAGS);
        frontLeftLimelight.setState(LimelightState.TAGS);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    frontCoralLimelight.sendImuData(
        robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    backTagLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    frontRightLimelight.sendImuData(
        robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    frontLeftLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
  }

  public void setClosestScoringReefTag(int tagID) {
    frontCoralLimelight.setClosestScoringReefTag(tagID);
    frontRightLimelight.setClosestScoringReefTag(tagID);
    frontLeftLimelight.setClosestScoringReefTag(tagID);
    backTagLimelight.setClosestScoringReefTag(tagID);
  }

  public boolean isAnyCameraOffline() {
    return frontCoralLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || backTagLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || frontRightLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || frontLeftLimelight.getCameraHealth() == CameraHealth.OFFLINE;
  }

  public boolean isAnyScoringTagLimelightOnline() {
    if ((frontLeftLimelight.getState() == LimelightState.TAGS
            || frontLeftLimelight.getState() == LimelightState.CLOSEST_REEF_TAG)
        && (frontLeftLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || frontLeftLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }
    if ((frontRightLimelight.getState() == LimelightState.TAGS
            || frontRightLimelight.getState() == LimelightState.CLOSEST_REEF_TAG)
        && (frontRightLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || frontRightLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }

    return false;
  }

  public boolean isAnyTagLimelightOnline() {
    if ((backTagLimelight.getState() == LimelightState.TAGS
            || backTagLimelight.getState() == LimelightState.CLOSEST_REEF_TAG)
        && (backTagLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || backTagLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }
    if ((frontLeftLimelight.getState() == LimelightState.TAGS
            || frontLeftLimelight.getState() == LimelightState.CLOSEST_REEF_TAG)
        && (frontLeftLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || frontLeftLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }
    if ((frontLeftLimelight.getState() == LimelightState.TAGS
            || frontLeftLimelight.getState() == LimelightState.CLOSEST_REEF_TAG)
        && (frontLeftLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || frontLeftLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }

    return false;
  }
}
