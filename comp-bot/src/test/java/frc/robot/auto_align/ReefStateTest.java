package frc.robot.auto_align;

import static org.assertj.core.api.Assertions.assertThat;

import com.google.common.collect.ImmutableList;
import org.junit.jupiter.api.Test;

final class ReefStateTest {
  private static final ImmutableList<ReefPipeLevel> TRACKED_LEVELS =
      ImmutableList.of(ReefPipeLevel.L2, ReefPipeLevel.L3, ReefPipeLevel.L4);

  @Test
  void markScoredTest() {
    var state = new ReefState();

    assertThat(state.isCoralScored(ReefPipe.PIPE_A, ReefPipeLevel.L4)).isFalse();

    state.markCoralScored(ReefPipe.PIPE_A, ReefPipeLevel.L4);

    for (var pipe : ReefPipe.values()) {
      for (var level : TRACKED_LEVELS) {
        var result = state.isCoralScored(pipe, level);
        if (pipe == ReefPipe.PIPE_A && level == ReefPipeLevel.L4) {
          assertThat(result).isTrue();
        } else {
          assertThat(result).isFalse();
        }
      }
    }
  }

  @Test
  void initialStateTest() {
    var state = new ReefState();

    for (var pipe : ReefPipe.values()) {
      for (var level : TRACKED_LEVELS) {
        assertThat(state.isCoralScored(pipe, level)).isFalse();
      }
    }
  }

  @Test
  void clearTest() {
    var state = new ReefState();

    state.markCoralScored(ReefPipe.PIPE_A, ReefPipeLevel.L4);
    state.markCoralScored(ReefPipe.PIPE_B, ReefPipeLevel.L4);

    state.clear();

    for (var pipe : ReefPipe.values()) {
      for (var level : TRACKED_LEVELS) {
        assertThat(state.isCoralScored(pipe, level)).isFalse();
      }
    }
  }

  @Test
  void doubleScoreTest() {
    var state = new ReefState();

    state.markCoralScored(ReefPipe.PIPE_A, ReefPipeLevel.L4);
    assertThat(state.isCoralScored(ReefPipe.PIPE_A, ReefPipeLevel.L4)).isTrue();

    // Shouldn't change state
    state.markCoralScored(ReefPipe.PIPE_A, ReefPipeLevel.L4);
    assertThat(state.isCoralScored(ReefPipe.PIPE_A, ReefPipeLevel.L4)).isTrue();
  }
}
