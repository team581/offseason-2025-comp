package frc.robot.auto_align;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.Pair;
import frc.robot.util.MathHelpers;
import org.junit.jupiter.api.Test;

public class ReefSideTest {
  private static void assertSidePoseCorect(boolean isRed, ReefSide side) {
    var pipe1Distance =
        side.pipe1
            .getPose(ReefPipeLevel.BASE, isRed)
            .getTranslation()
            .getDistance(side.getPose(isRed).getTranslation());
    var pipe2Distance =
        side.pipe2
            .getPose(ReefPipeLevel.BASE, isRed)
            .getTranslation()
            .getDistance(side.getPose(isRed).getTranslation());

    assertEquals(
        MathHelpers.roundTo(pipe1Distance, 0.01), MathHelpers.roundTo(pipe2Distance, 0.01));
  }

  private static Pair<ReefSide, ReefSide> getNeighbors(ReefSide side) {
    return switch (side) {
      case SIDE_CD -> new Pair<>(ReefSide.SIDE_AB, ReefSide.SIDE_EF);
      case SIDE_EF -> new Pair<>(ReefSide.SIDE_CD, ReefSide.SIDE_GH);
      case SIDE_GH -> new Pair<>(ReefSide.SIDE_EF, ReefSide.SIDE_IJ);
      case SIDE_IJ -> new Pair<>(ReefSide.SIDE_GH, ReefSide.SIDE_KL);
      case SIDE_KL -> new Pair<>(ReefSide.SIDE_IJ, ReefSide.SIDE_AB);
      case SIDE_AB -> new Pair<>(ReefSide.SIDE_KL, ReefSide.SIDE_CD);
    };
  }

  private static void assertSideSpacingCorrect(boolean isRed, ReefSide side) {
    var neighbors = getNeighbors(side);

    var sideTranslation = side.getPose(isRed).getTranslation();

    var distanceToPrevious =
        sideTranslation.getDistance(neighbors.getFirst().getPose(isRed).getTranslation());
    var distanceToNext =
        sideTranslation.getDistance(neighbors.getSecond().getPose(isRed).getTranslation());

    assertEquals(
        MathHelpers.roundTo(distanceToPrevious, 0.01), MathHelpers.roundTo(distanceToNext, 0.01));
  }

  @Test
  void sideAbRedTest() {
    assertSidePoseCorect(true, ReefSide.SIDE_AB);
    assertSideSpacingCorrect(true, ReefSide.SIDE_AB);
  }

  @Test
  void sideAbBlueTest() {
    assertSidePoseCorect(false, ReefSide.SIDE_AB);
    assertSideSpacingCorrect(false, ReefSide.SIDE_AB);
  }

  @Test
  void sideCdRedTest() {
    assertSidePoseCorect(true, ReefSide.SIDE_CD);
    assertSideSpacingCorrect(true, ReefSide.SIDE_CD);
  }

  @Test
  void sideCdBlueTest() {
    assertSidePoseCorect(false, ReefSide.SIDE_CD);
    assertSideSpacingCorrect(false, ReefSide.SIDE_CD);
  }

  @Test
  void sideEfRedTest() {
    assertSidePoseCorect(true, ReefSide.SIDE_EF);
    assertSideSpacingCorrect(true, ReefSide.SIDE_EF);
  }

  @Test
  void sideEfBlueTest() {
    assertSidePoseCorect(false, ReefSide.SIDE_EF);
    assertSideSpacingCorrect(false, ReefSide.SIDE_EF);
  }

  @Test
  void sideGhRedTest() {
    assertSidePoseCorect(true, ReefSide.SIDE_GH);
    assertSideSpacingCorrect(true, ReefSide.SIDE_GH);
  }

  @Test
  void sideGhBlueTest() {
    assertSidePoseCorect(false, ReefSide.SIDE_GH);
    assertSideSpacingCorrect(false, ReefSide.SIDE_GH);
  }

  @Test
  void sideIjRedTest() {
    assertSidePoseCorect(true, ReefSide.SIDE_IJ);
    assertSideSpacingCorrect(true, ReefSide.SIDE_IJ);
  }

  @Test
  void sideIjBlueTest() {
    assertSidePoseCorect(false, ReefSide.SIDE_IJ);
    assertSideSpacingCorrect(false, ReefSide.SIDE_IJ);
  }

  @Test
  void sideKlRedTest() {
    assertSidePoseCorect(true, ReefSide.SIDE_KL);
    assertSideSpacingCorrect(true, ReefSide.SIDE_KL);
  }

  @Test
  void sideKlBlueTest() {
    assertSidePoseCorect(false, ReefSide.SIDE_KL);
    assertSideSpacingCorrect(false, ReefSide.SIDE_KL);
  }
}
