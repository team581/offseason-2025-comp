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
  private final Limelight rightTagLimelight;
  private final Limelight leftTagLimelight;

  private final List<TagResult> tagResult = new ArrayList<>();
  private double robotHeading;
  private double pitch;
  private double angularVelocity;
  private double pitchRate;
  private double roll;
  private double rollRate;

  public VisionSubsystem(
      ImuSubsystem imu, Limelight rightTagLimelight, Limelight leftTagLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.rightTagLimelight = rightTagLimelight;
    this.leftTagLimelight = leftTagLimelight;
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
    var maybeRightResult = rightTagLimelight.getTagResult();
    var maybeLeftResult = leftTagLimelight.getTagResult();

    if (maybeRightResult.isPresent()) {
      tagResult.add(maybeRightResult.orElseThrow());
    }
    
    if (maybeLeftResult.isPresent()) {
      tagResult.add(maybeLeftResult.orElseThrow());
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
        rightTagLimelight.setState(LimelightState.TAGS);
        leftTagLimelight.setState(LimelightState.TAGS);
      }
      case CLOSEST_REEF_TAG -> {
        rightTagLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
        leftTagLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
      }
      case CORAL_DETECTION -> {
        rightTagLimelight.setState(LimelightState.TAGS);
        leftTagLimelight.setState(LimelightState.TAGS);
      }
      case ALGAE_DETECTION -> {
        rightTagLimelight.setState(LimelightState.TAGS);
        leftTagLimelight.setState(LimelightState.TAGS);
      }
    }
  }

  public Optional<GamePieceResult> getLollipopVisionResult() {
    // TODO: UPDATE WITH GP DETECTION LL
    return rightTagLimelight.getAlgaeResult();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    rightTagLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    leftTagLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
  }

  public void setClosestScoringReefTag(int tagID) {
    rightTagLimelight.setClosestScoringReefTag(tagID);
    leftTagLimelight.setClosestScoringReefTag(tagID);
  }

  public boolean isAnyCameraOffline() {
    // TODO: UPDATE WITH NEW GP LL
    return rightTagLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || leftTagLimelight.getCameraHealth() == CameraHealth.OFFLINE;
  }

  public boolean isAnyTagLimelightOnline() {
    if ((rightTagLimelight.getState() == LimelightState.TAGS
            || rightTagLimelight.getState() == LimelightState.CLOSEST_REEF_TAG)
        && (rightTagLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || rightTagLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }
    if ((leftTagLimelight.getState() == LimelightState.TAGS
            || leftTagLimelight.getState() == LimelightState.CLOSEST_REEF_TAG)
        && (leftTagLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || leftTagLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }

    return false;
  }
}
