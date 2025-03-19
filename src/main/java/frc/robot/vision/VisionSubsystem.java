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
  private final Limelight frontTagLimelight;
  private final Limelight backTagLimelight;

  private final List<TagResult> tagResult = new ArrayList<>();
  private double robotHeading;
  private double pitch;
  private double angularVelocity;
  private double pitchRate;
  private double roll;
  private double rollRate;

  public VisionSubsystem(
      ImuSubsystem imu, Limelight frontTagLimelight, Limelight backTagLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.frontTagLimelight = frontTagLimelight;
    this.backTagLimelight = backTagLimelight;
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
    var maybeBackResult = backTagLimelight.getTagResult();
    var maybeFrontResult = frontTagLimelight.getTagResult();
    if (maybeBackResult.isPresent()) {
      tagResult.add(maybeBackResult.orElseThrow());
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

  @Override
  protected void afterTransition(VisionState newState) {
    switch (newState) {
      case TAGS -> {
        frontTagLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
      }
      case CLOSEST_REEF_TAG -> {
        frontTagLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
        backTagLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
      }
      case CORAL_DETECTION -> {
        frontTagLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
      }
      case ALGAE_DETECTION -> {
        frontTagLimelight.setState(LimelightState.TAGS);
        backTagLimelight.setState(LimelightState.TAGS);
      }
    }
  }

  public Optional<GamePieceResult> getLollipopVisionResult() {
    // TODO: UPDATE WITH GP DETECTION LL
    return frontTagLimelight.getAlgaeResult();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    frontTagLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    backTagLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
  }

  public void setClosestScoringReefTag(int tagID) {
    frontTagLimelight.setClosestScoringReefTag(tagID);
    backTagLimelight.setClosestScoringReefTag(tagID);
  }

  public boolean isAnyCameraOffline() {
    // TODO: UPDATE WITH NEW GP LL
    return frontTagLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || backTagLimelight.getCameraHealth() == CameraHealth.OFFLINE;
  }

  public boolean isAnyTagLimelightOnline() {
    if ((frontTagLimelight.getState() == LimelightState.TAGS
            || frontTagLimelight.getState() == LimelightState.CLOSEST_REEF_TAG)
        && (frontTagLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || frontTagLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }
    if ((backTagLimelight.getState() == LimelightState.TAGS
            || backTagLimelight.getState() == LimelightState.CLOSEST_REEF_TAG)
        && (backTagLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || backTagLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }

    return false;
  }
}
