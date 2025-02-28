package frc.robot.auto_align.field_calibration;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.config.RobotConfig;
import frc.robot.elevator.ElevatorState;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.lights.LightsState;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.wrist.WristState;
import frc.robot.wrist.WristSubsystem;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

/**
 * Logs useful diagnostics to validate scoring setpoints during field calibration. Enabled with the
 * "FIELD_CALIBRATION" feature flag.
 *
 * <p>FIELD_CALIBRATION has the following changes:
 *
 * <ul>
 *   <li>Lights: green & yellow lights show whether scoring align & superstructure is good
 *   <li>Elevator: in coast mode while disabled
 *   <li>Wrist: assumes it is at bottom hardstop when code starts
 */
public class FieldCalibrationUtil {
  private static final double ELEVATOR_TOLERANCE = RobotConfig.get().elevator().tolerance();
  // Copied from WristSubsystem
  private static final double WRIST_TOLERANCE = 2.0;
  // Copied from TagAlign
  private static final double TRANSLATION_TOLERANCE = 0.05;

  private static final List<ReefPipeLevel> LEVELS =
      List.of(ReefPipeLevel.L2, ReefPipeLevel.L3, ReefPipeLevel.L4);

  private static Summary createSummary(
      ElevatorState wantedElevator,
      double actualElevator,
      WristState wantedWrist,
      double actualWrist,
      Translation2d wantedTranslation,
      Translation2d actualTranslation) {
    var elevatorError = wantedElevator.height - actualElevator;
    var wristError = wantedWrist.angle - actualWrist;
    var alignError = wantedTranslation.getDistance(actualTranslation);

    var elevatorState = MechanismState.OK;
    var wristState = MechanismState.OK;
    var alignOk = MathUtil.isNear(0, alignError, TRANSLATION_TOLERANCE);

    if (elevatorError > ELEVATOR_TOLERANCE) {
      elevatorState = MechanismState.TOO_LOW;
    } else if (elevatorError < -ELEVATOR_TOLERANCE) {
      elevatorState = MechanismState.TOO_HIGH;
    }

    if (wristError > WRIST_TOLERANCE) {
      wristState = MechanismState.TOO_LOW;
    } else if (wristError < -WRIST_TOLERANCE) {
      wristState = MechanismState.TOO_HIGH;
    }

    return new Summary(elevatorState, wristState, alignOk);
  }

  private static ElevatorState branchToElevator(ReefPipeLevel level, boolean centered) {
    return switch (level) {
      case L2 ->
          centered ? ElevatorState.CORAL_CENTERED_L2_PLACE : ElevatorState.CORAL_DISPLACED_L2_PLACE;
      case L3 ->
          centered ? ElevatorState.CORAL_CENTERED_L3_PLACE : ElevatorState.CORAL_DISPLACED_L3_PLACE;
      case L4 ->
          centered ? ElevatorState.CORAL_CENTERED_L4_PLACE : ElevatorState.CORAL_DISPLACED_L4_PLACE;
      default -> ElevatorState.UNTUNED;
    };
  }

  private static WristState branchToWrist(ReefPipeLevel level, boolean centered) {
    return switch (level) {
      case L2 ->
          centered
              ? WristState.CORAL_SCORE_CENTERED_PLACING_L2
              : WristState.CORAL_SCORE_DISPLACED_PLACING_L2;
      case L3 ->
          centered
              ? WristState.CORAL_SCORE_CENTERED_PLACING_L3
              : WristState.CORAL_SCORE_DISPLACED_PLACING_L3;
      case L4 ->
          centered
              ? WristState.CORAL_SCORE_CENTERED_PLACING_L4
              : WristState.CORAL_SCORE_DISPLACED_PLACING_L4;
      default -> WristState.UNTUNED;
    };
  }

  private final ElevatorSubsystem elevator;
  private final WristSubsystem wrist;
  private final LightsSubsystem lights;
  private final LocalizationSubsystem localization;

  public FieldCalibrationUtil(
      ElevatorSubsystem elevator,
      WristSubsystem wrist,
      LightsSubsystem lights,
      LocalizationSubsystem localization) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.lights = lights;
    this.localization = localization;
  }

  public void log() {
    var robotTranslation = localization.getPose().getTranslation();
    var bestRedPipe =
        Arrays.stream(ReefPipe.values())
            .min(
                Comparator.comparingDouble(
                    pipe ->
                        pipe.getPose(ReefPipeLevel.L4, true)
                            .getTranslation()
                            .getDistance(robotTranslation)))
            .orElseThrow();
    var bestBluePipe =
        Arrays.stream(ReefPipe.values())
            .min(
                Comparator.comparingDouble(
                    pipe ->
                        pipe.getPose(ReefPipeLevel.L4, false)
                            .getTranslation()
                            .getDistance(robotTranslation)))
            .orElseThrow();

    DogLog.log("FieldCalibration/Red/Best/Name", bestRedPipe);
    DogLog.log("FieldCalibration/Blue/Best/Name", bestBluePipe);

    var anyOk = false;

    for (var level : LEVELS) {
      DogLog.log("FieldCalibration/Red/Best/" + level.toString(), bestRedPipe.getPose(level, true));
      DogLog.log(
          "FieldCalibration/Blue/Best/" + level.toString(), bestBluePipe.getPose(level, false));

      var redDisplacedSummary = createSummary(bestRedPipe, true, level, false);
      var redCenteredSummary = createSummary(bestRedPipe, true, level, true);
      var blueDisplacedSummary = createSummary(bestBluePipe, false, level, false);
      var blueCenteredSummary = createSummary(bestBluePipe, false, level, true);

      anyOk =
          anyOk
              || redDisplacedSummary.isOk()
              || redCenteredSummary.isOk()
              || blueDisplacedSummary.isOk()
              || blueCenteredSummary.isOk();

      DogLog.log(
          "FieldCalibration/Red/Best/" + level.toString() + "/Displaced",
          redDisplacedSummary.format());
      DogLog.log(
          "FieldCalibration/Red/Best/" + level.toString() + "/Centered",
          redCenteredSummary.format());
      DogLog.log(
          "FieldCalibration/Blue/Best/" + level.toString() + "/Displaced",
          blueDisplacedSummary.format());
      DogLog.log(
          "FieldCalibration/Blue/Best/" + level.toString() + "/Centered",
          blueCenteredSummary.format());

      // Loop through all L2-4 pipes and log the scoring poses for each
      for (var pipe : ReefPipe.values()) {
        var branchSlug = pipe.toString() + "/" + level.toString() + "/Pose";
        DogLog.log("FieldCalibration/Red/" + branchSlug, pipe.getPose(level, true));
        DogLog.log("FieldCalibration/Blue/" + branchSlug, pipe.getPose(level, false));
      }
    }

    // Use lights to indicate we are in any valid scoring configuration
    lights.setDisabledState(
        anyOk ? LightsState.SCORE_ALIGN_READY : LightsState.SCORE_ALIGN_NOT_READY);
  }

  private Summary createSummary(
      ReefPipe pipe, boolean isRedAlliance, ReefPipeLevel level, boolean centered) {
    var actualElevator = elevator.getHeight();
    var actualWrist = wrist.getAngle();

    var wantedElevator = branchToElevator(level, centered);
    var wantedWrist = branchToWrist(level, centered);

    var wantedTranslation = pipe.getPose(level, isRedAlliance).getTranslation();
    var actualTranslation = localization.getPose().getTranslation();

    return createSummary(
        wantedElevator,
        actualElevator,
        wantedWrist,
        actualWrist,
        wantedTranslation,
        actualTranslation);
  }
}
