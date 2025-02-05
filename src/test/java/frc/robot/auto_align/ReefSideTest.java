package frc.robot.auto_align;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.util.MathHelpers;

public class ReefSideTest {
  private static void assertSidePoseCorect(boolean isRed, ReefSide side) {
    var pipe1Distance =
        side.pipe1
            .getPose(ReefPipeLevel.BASE)
            .getTranslation()
            .getDistance(side.getPose(isRed).getTranslation());
    var pipe2Distance =
        side.pipe2
            .getPose(ReefPipeLevel.BASE)
            .getTranslation()
            .getDistance(side.getPose(isRed).getTranslation());

    assertEquals(MathHelpers.roundTo(pipe1Distance, 0.01), MathHelpers.roundTo(pipe2Distance, 0.01));
  }

  @Test
  void sideAbRedTest() {
    assertSidePoseCorect(true, ReefSide.SIDE_AB);
  }

  @Test
  void sideAbBlueTest() {
    assertSidePoseCorect(false, ReefSide.SIDE_AB);
  }

  @Test
  void sideCdRedTest() {
    assertSidePoseCorect(true, ReefSide.SIDE_CD);
  }

  @Test
  void sideCdBlueTest() {
    assertSidePoseCorect(false, ReefSide.SIDE_CD);
  }

  @Test
  void sideEfRedTest() {
    assertSidePoseCorect(true, ReefSide.SIDE_EF);
  }

  @Test
  void sideEfBlueTest() {
    assertSidePoseCorect(false, ReefSide.SIDE_EF);
  }

  @Test
  void sideGhRedTest() {
    assertSidePoseCorect(true, ReefSide.SIDE_GH);
  }

  @Test
  void sideGhBlueTest() {
    assertSidePoseCorect(false, ReefSide.SIDE_GH);
  }

  @Test
  void sideIjRedTest() {
    assertSidePoseCorect(true, ReefSide.SIDE_IJ);
  }

  @Test
  void sideIjBlueTest() {
    assertSidePoseCorect(false, ReefSide.SIDE_IJ);
  }

  @Test
  void sideKlRedTest() {
    assertSidePoseCorect(true, ReefSide.SIDE_KL);
  }

  @Test
  void sideKlBlueTest() {
    assertSidePoseCorect(false, ReefSide.SIDE_KL);
  }
}
