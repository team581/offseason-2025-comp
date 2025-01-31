package frc.robot.auto_align;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class AutoAlignTest {
  @Test
  void scoreInNetOnBlueSideAndFacingNet() {
    // x range: 0-8.775 for blue, 8.775-17.55 for red
    var robotPose = new Pose2d(7, 0, Rotation2d.fromDegrees(23));

    var result = AutoAlign.shouldNetScoreForwards(robotPose);

    Assertions.assertEquals(true, result);
  }

  @Test
  void scoreInNetOnBlueSideAndFacingAway() {
    // x range: 0-8.775 for blue, 8.775-17.55 for red
    var robotPose = new Pose2d(3, 0, Rotation2d.fromDegrees(218));

    var result = AutoAlign.shouldNetScoreForwards(robotPose);

    Assertions.assertEquals(false, result);
  }

  @Test
  void scoreInNetOnRedSideAndFacingNet() {
    // x range: 0-8.775 for blue, 8.775-17.55 for red
    var robotPose = new Pose2d(9, 0, Rotation2d.fromDegrees(185));

    var result = AutoAlign.shouldNetScoreForwards(robotPose);

    Assertions.assertEquals(true, result);
  }

  @Test
  void scoreInNetOnRedSideAndFacingAway() {
    // x range: 0-8.775 for blue, 8.775-17.55 for red
    var robotPose = new Pose2d(13, 0, Rotation2d.fromDegrees(67));

    var result = AutoAlign.shouldNetScoreForwards(robotPose);

    Assertions.assertEquals(false, result);
  }

  @Test
  void scoreSideABRed() {
    var robotPose = new Pose2d(15.0, 5.0, Rotation2d.fromRadians(-Math.PI));

    var result = AutoAlign.getClosestReefSide(robotPose).getPose(true);

    Assertions.assertEquals(ReefSide.SIDE_AB.redPose, result);
  }

  @Test
  void scoreSideABBlue() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

    var robotPose = new Pose2d(3.658, 4.026, Rotation2d.fromRadians(0.0));

    var result = AutoAlign.getClosestReefSide(robotPose, false).getPose(false);

    Assertions.assertEquals(ReefSide.SIDE_AB.bluePose, result);
  }

  @Test
  void scoreSideCDRed() {
    var robotPose = new Pose2d(13.475, 3.306, Rotation2d.fromRadians(2.0944));

    var result = AutoAlign.getClosestReefSide(robotPose).getPose(true);

    Assertions.assertEquals(ReefSide.SIDE_CD.redPose, result);
  }

  @Test
  void scoreSideCDBlue() {

    var robotPose = new Pose2d(4.073, 4.746, Rotation2d.fromRadians(-1.0472));

    var result = AutoAlign.getClosestReefSide(robotPose, false).getPose(false);

    Assertions.assertEquals(ReefSide.SIDE_CD.bluePose, result);
  }

  @Test
  void scoreSideEFRed() {
    var robotPose = new Pose2d(12.643, 3.306, Rotation2d.fromRadians(1.0472));

    var result = AutoAlign.getClosestReefSide(robotPose).getPose(true);

    Assertions.assertEquals(ReefSide.SIDE_EF.redPose, result);
  }

  @Test
  void scoreSideEFBlue() {
    var robotPose = new Pose2d(4.905, 4.746, Rotation2d.fromRadians(-2.0944));

    var result = AutoAlign.getClosestReefSide(robotPose, false).getPose(false);

    Assertions.assertEquals(ReefSide.SIDE_EF.bluePose, result);
  }

  @Test
  void scoreSideGHRed() {
    var robotPose = new Pose2d(12.227, 4.026, Rotation2d.fromRadians(0.0));

    var result = AutoAlign.getClosestReefSide(robotPose).getPose(true);

    Assertions.assertEquals(ReefSide.SIDE_GH.redPose, result);
  }

  @Test
  void scoreSideGHBlue() {
    var robotPose = new Pose2d(5.321, 4.026, Rotation2d.fromRadians(Math.PI));

    var result = AutoAlign.getClosestReefSide(robotPose, false).getPose(false);

    Assertions.assertEquals(ReefSide.SIDE_GH.bluePose, result);
  }

  @Test
  void scoreSideIJRed() {
    var robotPose = new Pose2d(12.643, 4.746, Rotation2d.fromRadians(-1.0472));

    var result = AutoAlign.getClosestReefSide(robotPose).getPose(true);

    Assertions.assertEquals(ReefSide.SIDE_IJ.redPose, result);
  }

  @Test
  void scoreSideIJBlue() {
    var robotPose = new Pose2d(4.905, 3.306, Rotation2d.fromRadians(2.0944));

    var result = AutoAlign.getClosestReefSide(robotPose, false).getPose(false);

    Assertions.assertEquals(ReefSide.SIDE_IJ.bluePose, result);
  }

  @Test
  void scoreSideKLRed() {
    var robotPose = new Pose2d(13.474, 4.746, Rotation2d.fromRadians(-2.0944));

    var result = AutoAlign.getClosestReefSide(robotPose).getPose(true);

    Assertions.assertEquals(ReefSide.SIDE_KL.redPose, result);
  }

  @Test
  void scoreSideKLBlue() {
    var robotPose = new Pose2d(4.074, 3.306, Rotation2d.fromRadians(1.0472));

    var result = AutoAlign.getClosestReefSide(robotPose, false).getPose(false);

    Assertions.assertEquals(ReefSide.SIDE_KL.bluePose, result);
  }
}
