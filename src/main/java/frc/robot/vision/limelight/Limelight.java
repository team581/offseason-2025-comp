package frc.robot.vision.limelight;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.CameraStatus;
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
  private CameraStatus cameraStatus = CameraStatus.NO_TARGETS;
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

    var coralTX = LimelightHelpers.getTX(limelightTableName);
    var coralTY = LimelightHelpers.getTY(limelightTableName);
    var latency =
        LimelightHelpers.getLatency_Capture(limelightTableName)
            + LimelightHelpers.getLatency_Pipeline(limelightTableName);
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

    var purpleTX = LimelightHelpers.getTX(limelightTableName);
    var purpleTY = LimelightHelpers.getTY(limelightTableName);
    var latency =
        LimelightHelpers.getLatency_Capture(limelightTableName)
            + LimelightHelpers.getLatency_Pipeline(limelightTableName);
    var latencySeconds = latency / 1000.0;
    var timestamp = Timer.getFPGATimestamp() - latencySeconds;
    if (purpleTX == 0.0 || purpleTY == 0.0) {
      return Optional.empty();
    }

    DogLog.log("Vision/" + name + "/Purple/tx", purpleTX);
    DogLog.log("Vision/" + name + "/Purple/ty", purpleTY);

    return Optional.of(new PurpleResult(purpleTX, purpleTY, timestamp));
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
          interpolatedPose =
          InterpolatedVision.interpolatePose(getRawTagResult().get().pose(), cameraDataset);
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
      cameraStatus = CameraStatus.OFFLINE;
      return;
    }

    if (!result.isEmpty()) {
      cameraStatus = CameraStatus.GOOD;
      return;
    }
    cameraStatus = CameraStatus.NO_TARGETS;
  }

  public CameraStatus getCameraStatus() {
    return cameraStatus;
  }
}
