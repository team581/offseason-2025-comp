package frc.robot.vision.limelight;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.fms.FmsSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.CameraHealth;
import frc.robot.vision.interpolation.CameraDataset;
import frc.robot.vision.interpolation.InterpolatedVision;
import frc.robot.vision.results.CoralResult;
import frc.robot.vision.results.PurpleResult;
import frc.robot.vision.results.TagResult;
import java.util.Optional;

public class Limelight extends StateMachine<LimelightState> {
  private final String limelightTableName;
  private final String name;
  private final CameraDataset cameraDataset;
  private static final int[] RED_REEF_TAGS = {6, 7, 8, 9, 10, 11};
  private static final int[] BLUE_REEF_TAGS = {17, 18, 19, 20, 21, 22};

  private CameraHealth cameraHealth = CameraHealth.NO_TARGETS;
  private double limelightHeartbeat = -1;
  private final Timer limelightTimer = new Timer();

  private Pose2d interpolatedPose = new Pose2d();

  public Limelight(String name, LimelightState initialState, CameraDataset cameraDataset) {
    super(SubsystemPriority.VISION, initialState);
    limelightTableName = "limelight-" + name;
    this.name = name;
    limelightTimer.start();
    this.cameraDataset = cameraDataset;
  }

  public void sendImuData(
      double robotHeading,
      double angularVelocity,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {
    if (getState() == LimelightState.TAGS) {
      return;
    }
    LimelightHelpers.SetRobotOrientation(
        limelightTableName, robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
  }

  public void setState(LimelightState state) {
    setStateFromRequest(state);
  }

  public Optional<TagResult> getInterpolatedTagResult() {
    var rawTagResult = getRawTagResult();

    if (rawTagResult.isEmpty()) {
      return Optional.empty();
    }

    return Optional.of(new TagResult(interpolatedPose, rawTagResult.get().timestamp()));
  }

  private Optional<TagResult> getRawTagResult() {
    if (getState() != LimelightState.TAGS) {
      return Optional.empty();
    }

    var estimatePose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightTableName);

    if (estimatePose == null) {
      return Optional.empty();
    }

    DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", estimatePose.pose);

    if (estimatePose.tagCount == 0) {
      return Optional.empty();
    }

    // This prevents pose estimator from having crazy poses if the Limelight loses power
    if (estimatePose.pose.getX() == 0.0 && estimatePose.pose.getY() == 0.0) {
      return Optional.empty();
    }

    return Optional.of(new TagResult(estimatePose.pose, estimatePose.timestampSeconds));
  }

  private Optional<CoralResult> getRawCoralResult() {
    if (getState() != LimelightState.CORAL) {
      return Optional.empty();
    }
    var t2d = LimelightHelpers.getT2DArray(limelightTableName);
    if (t2d.length == 0) {
      return Optional.empty();
    }
    var coralTX = t2d[4];
    var coralTY = t2d[5];
    var latency = t2d[2] + t2d[3];
    var latencySeconds = latency / 1000.0;
    var timestamp = Timer.getFPGATimestamp() - latencySeconds;
    if (coralTX == 0.0 || coralTY == 0.0) {
      return Optional.empty();
    }

    DogLog.log("Vision/" + name + "/Coral/tx", coralTX);
    DogLog.log("Vision/" + name + "/Coral/ty", coralTY);

    return Optional.of(new CoralResult(coralTX, coralTY, timestamp));
  }

  private Optional<PurpleResult> getRawPurpleResult() {
    if (getState() != LimelightState.PURPLE) {
      return Optional.empty();
    }
    var t2d = LimelightHelpers.getT2DArray(limelightTableName);
    if (t2d.length == 0) {
      return Optional.empty();
    }
    var purpleTX = t2d[4];
    var purpleTY = t2d[5];
    var latency = t2d[2] + t2d[3];
    var latencySeconds = latency / 1000.0;
    var timestamp = Timer.getFPGATimestamp() - latencySeconds;
    if (purpleTX == 0.0 || purpleTY == 0.0) {
      return Optional.empty();
    }

    DogLog.log("Vision/" + name + "/Purple/tx", purpleTX);
    DogLog.log("Vision/" + name + "/Purple/ty", purpleTY);

    return Optional.of(new PurpleResult(purpleTX, purpleTY, timestamp));
  }

  private int[] getAllianceBasedReefTagIDs() {
    return FmsSubsystem.isRedAlliance() ? RED_REEF_TAGS : BLUE_REEF_TAGS;
  }

  private Optional<TagResult> tagResult = Optional.empty();
  private Optional<CoralResult> coralResult = Optional.empty();
  private Optional<PurpleResult> purpleResult = Optional.empty();

  @Override
  protected void collectInputs() {
    tagResult = getRawTagResult();
    coralResult = getRawCoralResult();
    purpleResult = getRawPurpleResult();
    if (getState() == LimelightState.TAGS) {
      var maybeInterpolatedPose = getInterpolatedTagResult();
      if (maybeInterpolatedPose.isPresent()) {
        interpolatedPose =
            InterpolatedVision.interpolatePose(maybeInterpolatedPose.get().pose(), cameraDataset);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    LimelightHelpers.setPipelineIndex(limelightTableName, getState().pipelineIndex);

    switch (getState()) {
      case TAGS -> updateHealth(tagResult);
      case CORAL -> updateHealth(coralResult);
      case PURPLE -> updateHealth(purpleResult);
      case REEF_TAGS -> {
        LimelightHelpers.SetFiducialIDFiltersOverride(
            limelightTableName, getAllianceBasedReefTagIDs());
        tagResult = getRawTagResult();
        updateHealth(tagResult);
      }
      default -> {}
    }
  }

  private void updateHealth(Optional<?> result) {
    var newHeartbeat = LimelightHelpers.getLimelightNTDouble(limelightTableName, "hb");

    if (limelightHeartbeat != newHeartbeat) {
      limelightTimer.restart();
    }
    limelightHeartbeat = newHeartbeat;

    if (limelightTimer.hasElapsed(5)) {
      cameraHealth = CameraHealth.OFFLINE;
      return;
    }

    if (!result.isEmpty()) {
      cameraHealth = CameraHealth.GOOD;
      return;
    }
    cameraHealth = CameraHealth.NO_TARGETS;
  }

  public CameraHealth getCameraHealth() {
    return cameraHealth;
  }
}
