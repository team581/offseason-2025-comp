package frc.robot.auto_align.purple_align;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.util.MathHelpers;
import frc.robot.vision.limelight.Limelight;

public class PurpleAlign {
  private final Limelight purpleCamera;

  private static final double PURPLE_SIDEWAYS_KP = 1.0;
  private static final double SEEN_PURPLE_TIMEOUT = 3.0;



  private double lastTimeSeen = 0.0;
  private boolean seenPurple = false;

  public PurpleAlign(Limelight purpleCamera) {
    this.purpleCamera = purpleCamera;
  }

  public PurpleAlignState getPurpleState() {
    var maybeResult = purpleCamera.getPurpleResult();
    if (maybeResult.isEmpty()) {
      if (lastTimeSeen < Timer.getFPGATimestamp() - SEEN_PURPLE_TIMEOUT) {
        seenPurple = false;
        return PurpleAlignState.NO_PURPLE;
      }
      return PurpleAlignState.VISIBLE_NOT_CENTERED;
    }

    var result = maybeResult.get();

    if (MathUtil.isNear(0, result.ty(), 0.5)) {
      lastTimeSeen = Timer.getFPGATimestamp();
      seenPurple = true;
      return PurpleAlignState.CENTERED;
    }

    lastTimeSeen = Timer.getFPGATimestamp();
    seenPurple = true;
    return PurpleAlignState.VISIBLE_NOT_CENTERED;
  }

  public boolean seenPurple() {
    return seenPurple;
  }

  public ChassisSpeeds getPurpleAlignChassisSpeeds(double robotHeading) {
    var maybeResult = purpleCamera.getPurpleResult();
    if (maybeResult.isEmpty()) {
      return new ChassisSpeeds();
    }
    var rawAngle = maybeResult.get().ty();
    DogLog.log("PurpleAlignment/Purple/RawAngleTY", rawAngle);
    var rawAngleRadians = Units.degreesToRadians(rawAngle);
    var rawAngleTranslation = new Translation2d(0, rawAngleRadians);
    var rotatedAngleTranslation =
        rawAngleTranslation.rotateBy(Rotation2d.fromDegrees(robotHeading));
    var xError = rotatedAngleTranslation.getX();
    var yError = rotatedAngleTranslation.getY();

    var xEffort = xError * PURPLE_SIDEWAYS_KP;
    var yEffort = yError * PURPLE_SIDEWAYS_KP;
    DogLog.log("PurpleAlignment/Purple/XEffort", xEffort);
    DogLog.log("PurpleAlignment/Purple/YEffort", yEffort);
    return new ChassisSpeeds(xEffort, yEffort, 0.0);
  }


  public boolean canUsePurple() {
    // TODO: Implement
    return false;
  }
}
