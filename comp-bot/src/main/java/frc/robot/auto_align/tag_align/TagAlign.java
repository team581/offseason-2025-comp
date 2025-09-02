package frc.robot.auto_align.tag_align;

import com.google.common.collect.ImmutableList;
import com.team581.math.MathHelpers;
import com.team581.math.PolarChassisSpeeds;
import com.team581.trailblazer.constraints.AutoConstraintOptions;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefSide;
import frc.robot.auto_align.ReefState;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.config.FeatureFlags;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import java.util.Comparator;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;

public class TagAlign {
  public static final ImmutableList<ReefPipe> ALL_REEF_PIPES =
      ImmutableList.copyOf(ReefPipe.values());
  public static final ImmutableList<ReefSide> ALL_REEF_SIDES =
      ImmutableList.copyOf(ReefSide.values());
  public static final double L1_TRACKING_TIMEOUT = 15.0;

  private static final ProfiledPIDController REEF_PIPE_ROTATION_CONTROLLER =
      new ProfiledPIDController(
          5.75,
          0.0,
          0.0,
          new Constraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(1.0)));
  private static final ProfiledPIDController ALGAE_ROTATION_CONTROLLER =
      new ProfiledPIDController(
          5.75,
          0.0,
          0.0,
          new Constraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(1.0)));
  private  static final ProfiledPIDController L1_ROTATION_CONTROLLER =
      new ProfiledPIDController(
          5.75,
          0.0,
          0.0,
          new Constraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(1.0)));

  private static final ProfiledPIDController REEF_PIPE_TRANSLATION_CONTROLLER =
      new ProfiledPIDController(
          4.0,
          0.0,
          0.0,
          new Constraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(1.0)));

  private  static final ProfiledPIDController ALGAE_TRANSLATION_CONTROLLER =
      new ProfiledPIDController(
          4.0,
          0.0,
          0.0,
          new Constraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(1.0)));

  private static final ProfiledPIDController L1_TRANSLATION_CONTROLLER =
      new ProfiledPIDController(4.0, 0.0, 0.0, new Constraints(3.0, 2.0));

  private static final InterpolatingDoubleTreeMap CORAL_TX_TO_L1_OFFSET =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(2.66, 2.943), Map.entry(0.0, 0.0), Map.entry(-11.0, -2.943));

  private static final DoubleSubscriber TRANSLATION_GOOD_THRESHOLD =
      DogLog.tunable("AutoAlign/IsAlignedTranslation", Units.inchesToMeters(1.0));
  private static final DoubleSubscriber ROTATION_GOOD_THRESHOLD =
      DogLog.tunable("AutoAlign/IsAlignedRotation", 3.0);

  private static final DoubleSubscriber NEAR_ROTATION_GOAL =
      DogLog.tunable("AutoAlign/IsAlignedRotation", 10.0);

  private static final DoubleSubscriber NEED_TO_MOVE_TRANSLATION_THRESHOLD =
      DogLog.tunable("AutoAlign/NeedMoveTranslation", Units.inchesToMeters(1.5));
  private static final DoubleSubscriber NEED_TO_MOVE_ROTATION_THRESHOLD =
      DogLog.tunable("AutoAlign/NeedMoveRotation", 3.0);

  private static final DoubleSubscriber IN_RANGE_TRANSLATION_THRESHOLD =
      DogLog.tunable("AutoAlign/InRangeTranslation", Units.inchesToMeters(1.5));
  private static final DoubleSubscriber IN_RANGE_ROTATION_THRESHOLD =
      DogLog.tunable("AutoAlign/InRangeRotation", 3.0);

  private static final double IDEAL_L1_OFFSET = Units.inchesToMeters(0.5);
  private static final Transform2d LEFT_L1_TRANSFORM =
      new Transform2d(0, IDEAL_L1_OFFSET, Rotation2d.fromDegrees(0));
  private static final Transform2d RIGHT_L1_TRANSFORM =
      new Transform2d(0, -IDEAL_L1_OFFSET, Rotation2d.fromDegrees(0));

  private static final double MAX_SPEED = 2.0;
  private static final double MAX_ROTATION_SPEED = Units.rotationsToRadians(3.0);
  private static final double PIPE_SWITCH_TIMEOUT = 0.5;

  private final AlignmentCostUtil alignmentCostUtil;
  private final LocalizationSubsystem localization;
  private final ReefState reefState = new ReefState();

  private ReefPipeLevel pipeLevel = ReefPipeLevel.RAISING;
  private ReefPipeLevel preferedScoringLevel = ReefPipeLevel.L4;
  private RobotScoringSide robotScoringSide = RobotScoringSide.RIGHT;
  private Optional<ReefPipe> reefPipeOverride = Optional.empty();
  private double rawControllerXValue = 0.0;
  private double rawControllerYValue = 0.0;
  private double lastPipeSwitchTimestamp = 0.0;
  private boolean aligned = false;
  private boolean translationGood = false;
  private boolean rotationGood = false;
  private boolean resetReefPipeNextLoop = false;
  private boolean resetAlgaeNextLoop = false;
  private boolean resetL1NextLoop = false;

  private OptionalDouble coralL1Offset = OptionalDouble.empty();
  private final Pose2d lastReefPipeTargetPose = new Pose2d();
  private final Pose2d lastL1TargetPose = new Pose2d();
  private final Pose2d lastAlgaeTargetPose = new Pose2d();

  private final LinearFilter l1AdjustmentFilter = LinearFilter.movingAverage(7);

  private static final DoubleSubscriber FEED_FORWARD = DogLog.tunable("AutoAlign/FeedForward", 0.0);

  private boolean pipeSwitchActive = false;

  private double lastAddedTimestamp = 0.0;

  public TagAlign(SwerveSubsystem swerve, LocalizationSubsystem localization) {
    this.localization = localization;
    alignmentCostUtil = new AlignmentCostUtil(localization, swerve, reefState, robotScoringSide);
    ALGAE_ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    L1_ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    REEF_PIPE_ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setLevel(ReefPipeLevel level, ReefPipeLevel preferredLevel, RobotScoringSide side) {
    this.robotScoringSide = side;
    alignmentCostUtil.setSide(robotScoringSide);
    this.pipeLevel = level;
    this.preferedScoringLevel = preferredLevel;
  }

  public void setPipeOveride(ReefPipe pipe) {
    this.reefPipeOverride = Optional.of(pipe);
  }

  public void setControllerValues(double controllerXValue, double controllerYValue) {
    rawControllerXValue = controllerXValue;
    rawControllerYValue = controllerYValue;
    checkControllerForSwitch();
  }

  public void setCoralL1Offset(OptionalDouble tx) {
    var expired = Timer.getFPGATimestamp() - lastAddedTimestamp > L1_TRACKING_TIMEOUT;
    if (expired) {
      coralL1Offset = OptionalDouble.empty();
    }
    if (tx.isEmpty()) {
      return;
    }

    var offset = CORAL_TX_TO_L1_OFFSET.get(tx.getAsDouble());
    lastAddedTimestamp = Timer.getFPGATimestamp();
    if (coralL1Offset.isEmpty()) {
      for (int i = 0; i < 7; i++) {
        l1AdjustmentFilter.calculate(offset);
      }
    }
    coralL1Offset = OptionalDouble.of(l1AdjustmentFilter.calculate(offset));
  }

  private void checkControllerForSwitch() {
    if (!DriverStation.isTeleop()) {
      return;
    }
    if (pipeSwitchActive
        && (Timer.getFPGATimestamp() > (lastPipeSwitchTimestamp + PIPE_SWITCH_TIMEOUT))
        && rawControllerXValue == 0.0) {
      pipeSwitchActive = false;
    }
    if (pipeSwitchActive) {
      return;
    }
    if ((Math.hypot(rawControllerXValue, rawControllerYValue) > 0.5)) {
      var storedPipe = getBestPipe();
      pipeSwitchActive = true;
      lastPipeSwitchTimestamp = Timer.getFPGATimestamp();

      var inputVector = new Translation2d(rawControllerXValue, -rawControllerYValue);
      var viewOffset = 0;
      if (FmsSubsystem.isRedAlliance()) {
        viewOffset = 180;
      }

      var sideAngle = ReefSide.fromPipe(storedPipe).getPose(FmsSubsystem.isRedAlliance());

      var rotatedVector =
          inputVector.rotateBy(
              Rotation2d.fromDegrees((viewOffset - sideAngle.getRotation().getDegrees())));
      var rotatedVectorLeft = rotatedVector.getX() < 0;
      ReefPipe leftPipe =
          switch (storedPipe) {
            case PIPE_A, PIPE_B -> ReefPipe.PIPE_A;
            case PIPE_C, PIPE_D -> ReefPipe.PIPE_C;
            case PIPE_E, PIPE_F -> ReefPipe.PIPE_E;
            case PIPE_G, PIPE_H -> ReefPipe.PIPE_G;
            case PIPE_I, PIPE_J -> ReefPipe.PIPE_I;
            case PIPE_K, PIPE_L -> ReefPipe.PIPE_K;
          };
      ReefPipe rightPipe =
          switch (storedPipe) {
            case PIPE_A, PIPE_B -> ReefPipe.PIPE_B;
            case PIPE_C, PIPE_D -> ReefPipe.PIPE_D;
            case PIPE_E, PIPE_F -> ReefPipe.PIPE_F;
            case PIPE_G, PIPE_H -> ReefPipe.PIPE_H;
            case PIPE_I, PIPE_J -> ReefPipe.PIPE_J;
            case PIPE_K, PIPE_L -> ReefPipe.PIPE_L;
          };
      var partnerPipe = ReefPipe.PIPE_A;
      if (rotatedVectorLeft) {
        DogLog.timestamp("AutoAlign/PipeSwitch/Left");
        partnerPipe = leftPipe;
      } else {
        DogLog.timestamp("AutoAlign/PipeSwitch/Right");
        partnerPipe = rightPipe;
      }
      reefState.removeCoral(partnerPipe, preferedScoringLevel);
      setPipeOveride(partnerPipe);
    }
  }

  public boolean isAligned(ReefPipe pipe) {
    if (pipeLevel.equals(ReefPipeLevel.RAISING) || pipeLevel.equals(ReefPipeLevel.BACK_AWAY)) {
      return false;
    }
    var robotPose = localization.getPose();
    var targetPose = getUsedScoringPose(pipe);

    translationGood =
        (robotPose.getTranslation().getDistance(targetPose.getTranslation())
            <= TRANSLATION_GOOD_THRESHOLD.get());
    rotationGood =
        MathUtil.isNear(
            targetPose.getRotation().getDegrees(),
            robotPose.getRotation().getDegrees(),
            ROTATION_GOOD_THRESHOLD.get(),
            -180.0,
            180.0);

    DogLog.log("AutoAlign/TranslationGood", translationGood);
    DogLog.log("AutoAlign/RotationGood", rotationGood);

    return translationGood && rotationGood;
  }

  public boolean isNearRotationGoal(ReefPipe pipe) {
    var robotPose = localization.getPose();
    var scoringPoseFieldRelative = getUsedScoringPose(pipe);
    var rotationGood =
        MathUtil.isNear(
            scoringPoseFieldRelative.getRotation().getDegrees(),
            robotPose.getRotation().getDegrees(),
            NEAR_ROTATION_GOAL.get(),
            -180.0,
            180.0);
    return rotationGood;
  }

  public boolean needToMove(Pose2d goal) {
    var robotPose = localization.getPose();
    var translationBad =
        (robotPose.getTranslation().getDistance(goal.getTranslation())
            > NEED_TO_MOVE_TRANSLATION_THRESHOLD.get());
    var rotationBad =
        !MathUtil.isNear(
            goal.getRotation().getDegrees(),
            robotPose.getRotation().getDegrees(),
            NEED_TO_MOVE_ROTATION_THRESHOLD.get(),
            -180.0,
            180.0);
    return translationBad || rotationBad;
  }

  public boolean inRange(Pose2d goal) {
    var robotPose = localization.getPose();
    var translationGood =
        (robotPose.getTranslation().getDistance(goal.getTranslation())
            <= IN_RANGE_TRANSLATION_THRESHOLD.get());
    var rotationGood =
        MathUtil.isNear(
            goal.getRotation().getDegrees(),
            robotPose.getRotation().getDegrees(),
            IN_RANGE_ROTATION_THRESHOLD.get(),
            -180.0,
            180.0);
    return translationGood && rotationGood;
  }

  public void markScored(ReefPipe pipe) {
    if (preferedScoringLevel.equals(ReefPipeLevel.L1)) {
      resetL1();
    }
    reefState.markCoralScored(pipe, preferedScoringLevel);
  }

  public void markAlgaeRemoved(ReefSide side) {
    reefState.markAlgaeRemoved(side);
  }

  public boolean isAlgaeRemoved(ReefSide side) {
    return reefState.isAlgaeRemoved(side);
  }

  public void reset() {
    resetL1NextLoop = true;
    resetAlgaeNextLoop = true;
    resetReefPipeNextLoop = true;
  }

  private void resetL1() {
    coralL1Offset = OptionalDouble.empty();
  }

  public void clearReefState() {
    reefState.clear();
  }

  public Pose2d getUsedScoringPose(ReefPipe pipe) {
    return getUsedScoringPose(pipe, robotScoringSide);
  }

  public Pose2d getUsedScoringPose(ReefPipe pipe, RobotScoringSide side) {
    if (preferedScoringLevel.equals(ReefPipeLevel.L1)) {
      var reefPose = ReefSide.fromPipe(pipe).getPose(localization.getPose());
      var pipePose = pipe.getPose(pipeLevel, side, localization.getPose());
      var rotatedPipePose =
          pipePose.rotateAround(
              reefPose.getTranslation(),
              Rotation2d.fromDegrees(360 - reefPose.getRotation().getDegrees()));
      var isLeft = rotatedPipePose.getY() > reefPose.getY();
      if (isLeft) {
        return pipePose
            .transformBy(LEFT_L1_TRANSFORM)
            .transformBy(
                new Transform2d(
                    0, Units.inchesToMeters(coralL1Offset.orElse(0.0)), Rotation2d.fromDegrees(0)));
      }
      return pipePose
          .transformBy(RIGHT_L1_TRANSFORM)
          .transformBy(
              new Transform2d(
                  0, Units.inchesToMeters(coralL1Offset.orElse(0.0)), Rotation2d.fromDegrees(0)));
    }
    return pipe.getPose(pipeLevel, side, localization.getPose());
  }

  /** Returns the best reef pipe for scoring, based on the robot's current state. */
  public ReefPipe getBestPipe() {
    if ((DriverStation.isAutonomous() || pipeSwitchActive) && reefPipeOverride.isPresent()) {
      return reefPipeOverride.orElseThrow();
    }
    var level = pipeLevel;
    var robotPose = localization.getPose();
    if (pipeLevel.equals(ReefPipeLevel.L1)) {
      var closestTwoPipes =
          ALL_REEF_PIPES.stream()
              .sorted(
                  Comparator.comparingDouble(
                      p ->
                          p.getPose(pipeLevel, robotScoringSide, localization.getPose())
                              .getTranslation()
                              .getDistance(localization.getPose().getTranslation())))
              .limit(2)
              .toList();

      return closestTwoPipes.stream()
          .min(Comparator.comparingDouble(p -> reefState.getL1Count(p)))
          .orElse(getClosestPipe());
    }
    if (pipeLevel.equals(ReefPipeLevel.BACK_AWAY)) {
      return ALL_REEF_PIPES.stream()
          .min(
              Comparator.comparingDouble(
                  pipe ->
                      robotPose
                          .getTranslation()
                          .getDistance(
                              pipe.getPose(ReefPipeLevel.BACK_AWAY, robotScoringSide, robotPose)
                                  .getTranslation())))
          .orElseThrow();
    }
    if (pipeLevel.equals(ReefPipeLevel.RAISING)) {
      level = preferedScoringLevel;
    }
    return ALL_REEF_PIPES.stream()
        .min(alignmentCostUtil.getReefPipeComparator(level))
        .orElseThrow();
  }

  public int getL1ScoredCount(ReefPipe pipe) {
    return reefState.getL1Count(pipe);
  }

  public ReefPipe getClosestPipe() {
    var robotPose = localization.getPose();
    return ALL_REEF_PIPES.stream()
        .min(
            Comparator.comparingDouble(
                pipe ->
                    robotPose
                        .getTranslation()
                        .getDistance(
                            pipe.getPose(pipeLevel, robotScoringSide, robotPose).getTranslation())))
        .orElseThrow();
  }

  public ReefSide getBestAlgaeSide() {
    return ALL_REEF_SIDES.stream().min(alignmentCostUtil.getAlgaeComparator()).orElseThrow();
  }

  public PolarChassisSpeeds getReefPipeAlignmentChassisSpeeds(
      Pose2d targetPose,
      Pose2d currentPose,
      AutoConstraintOptions constraints,
      PolarChassisSpeeds currentSpeeds) {
    var reset = false;
    if (resetReefPipeNextLoop) {
      reset = true;
    }
    resetReefPipeNextLoop = false;
    return getPoseAlignmentChassisSpeeds(
        targetPose,
        currentPose,
        REEF_PIPE_TRANSLATION_CONTROLLER,
        REEF_PIPE_ROTATION_CONTROLLER,
        constraints,
        currentSpeeds,
        reset);
  }

  public PolarChassisSpeeds getAlgaeAlignmentChassisSpeeds(
      Pose2d targetPose,
      Pose2d currentPose,
      AutoConstraintOptions constraints,
      PolarChassisSpeeds currentSpeeds) {
        var reset = false;
        if (resetAlgaeNextLoop) {
          reset = true;
        }
        resetAlgaeNextLoop = false;
    return getPoseAlignmentChassisSpeeds(
        targetPose,
        currentPose,
        ALGAE_TRANSLATION_CONTROLLER,
        ALGAE_ROTATION_CONTROLLER,
        constraints,
        currentSpeeds,
        reset);
  }

  public PolarChassisSpeeds getL1AlignmentChassisSpeeds(
      Pose2d targetPose,
      Pose2d currentPose,
      AutoConstraintOptions constraints,
      PolarChassisSpeeds currentSpeeds) {
        var reset = false;
        if (resetL1NextLoop) {
          reset = true;
        }
        resetL1NextLoop = false;
    return getPoseAlignmentChassisSpeeds(
        targetPose,
        currentPose,
        L1_TRANSLATION_CONTROLLER,
        L1_ROTATION_CONTROLLER,
        constraints,
        currentSpeeds,
        reset);
  }

  private PolarChassisSpeeds getPoseAlignmentChassisSpeeds(
      Pose2d targetPose,
      Pose2d currentPose,
      ProfiledPIDController translationController,
      ProfiledPIDController rotationController,
      AutoConstraintOptions constraints,
      PolarChassisSpeeds currentSpeeds,
      boolean reset) {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    if (FeatureFlags.AUTO_ALIGN_DEADBAND.getAsBoolean()) {
      if (aligned || inRange(targetPose)) {
        if (needToMove(targetPose)) {
          aligned = false;
        } else {
          aligned = true;
          return new PolarChassisSpeeds();
        }
      }
    }

    // Calculate x and y velocities
    double distanceToGoalMeters =
        currentPose.getTranslation().getDistance(targetPose.getTranslation());

    if (reset) {
      DogLog.timestamp("AutoAlign/ResetControllers");
      rotationController.reset(
          currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
      rotationController.setGoal(targetPose.getRotation().getRadians());
      translationController.reset(
                    distanceToGoalMeters,
                    Math.min(
                            0.0,
                            -new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
                                    .rotateBy(targetPose.getTranslation()
                                            .minus(currentPose.getTranslation())
                                            .getAngle()
                                            .unaryMinus())
                                    .getX()));
            translationController.setGoal(0);
    }

    var driveVelocityMagnitude =
        translationController.calculate(
            distanceToGoalMeters,
            new State(0, 0),
            new Constraints(constraints.maxLinearVelocity(), constraints.maxLinearAcceleration()));

    if (!translationGood) {
      driveVelocityMagnitude += Math.copySign(FEED_FORWARD.get(), driveVelocityMagnitude);
    }

    DogLog.log("AutoAlign/setpoint", Units.radiansToDegrees(distanceToGoalMeters));
    if (MathUtil.isNear(
        targetPose.getRotation().getRadians(), rotationController.getSetpoint().position, 1e-6)) {}

    ;

    var rotationSpeed =
        rotationController.calculate(
            currentPose.getRotation().getRadians(),
            new State(targetPose.getRotation().getRadians(), 0),
            constraints.getAngularConstraints());

    var driveDirection = MathHelpers.getDriveDirection(currentPose, targetPose);

    DogLog.log("AutoAlign/DistanceToGoal", distanceToGoalMeters);
    DogLog.log("AutoAlign/DriveVelocityMagnitude", driveVelocityMagnitude);
    DogLog.log("AutoAlign/RotationSpeed", rotationSpeed);
    DogLog.log("AutoAlign/DriveDirection", driveDirection.getDegrees());

    var speeds = new PolarChassisSpeeds(driveVelocityMagnitude, driveDirection, rotationSpeed);

    return speeds;
  }
}
