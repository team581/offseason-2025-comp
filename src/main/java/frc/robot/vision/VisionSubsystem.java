package frc.robot.vision;

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
  private final Limelight baseTagLimelight;

  private final List<TagResult> tagResult = new ArrayList<>();
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
      Limelight backTagLimelight,
      Limelight baseTagLimelight) {
    super(SubsystemPriority.VISION, VisionState.DEFAULT_STATE);
    this.imu = imu;
    this.elevatorPurpleLimelight = elevatorPurpleLimelight;
    this.frontCoralLimelight = frontCoralLimelight;
    this.backTagLimelight = backTagLimelight;
    this.baseTagLimelight = baseTagLimelight;
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
    var maybeTopResult = elevatorPurpleLimelight.getTagResult();
    var maybeBottomResult = frontCoralLimelight.getTagResult();
    var maybeBackResult = backTagLimelight.getTagResult();
    var maybeBaseResult = baseTagLimelight.getTagResult();

    if (maybeTopResult.isPresent()) {
      tagResult.add(maybeTopResult.get());
    }

    if (maybeBottomResult.isPresent()) {
      tagResult.add(maybeBottomResult.get());
    }

    if (maybeBackResult.isPresent()) {
      tagResult.add(maybeBackResult.get());
    }

    if (maybeBaseResult.isPresent()) {
      tagResult.add(maybeBaseResult.get());
    }
  }

  public List<TagResult> getTagResult() {
    return tagResult;
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    elevatorPurpleLimelight.sendImuData(
        robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    frontCoralLimelight.sendImuData(
        robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    backTagLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    baseTagLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
  }

  public boolean isAnyScoringTagLimelightOnline() {
    if ((frontCoralLimelight.getState() == LimelightState.TAGS
            || frontCoralLimelight.getState() == LimelightState.REEF_TAGS)
        && (frontCoralLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || frontCoralLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }
    if ((baseTagLimelight.getState() == LimelightState.TAGS
            || baseTagLimelight.getState() == LimelightState.REEF_TAGS)
        && (baseTagLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || baseTagLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }

    return false;
  }

  public boolean isAnyTagLimelightOnline() {
    if ((backTagLimelight.getState() == LimelightState.TAGS
            || backTagLimelight.getState() == LimelightState.REEF_TAGS)
        && (backTagLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || backTagLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }
    if ((frontCoralLimelight.getState() == LimelightState.TAGS
            || frontCoralLimelight.getState() == LimelightState.REEF_TAGS)
        && (frontCoralLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || frontCoralLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }
    if ((baseTagLimelight.getState() == LimelightState.TAGS
            || baseTagLimelight.getState() == LimelightState.REEF_TAGS)
        && (baseTagLimelight.getCameraHealth() == CameraHealth.NO_TARGETS
            || baseTagLimelight.getCameraHealth() == CameraHealth.GOOD)) {
      return true;
    }

    return false;
  }
}
