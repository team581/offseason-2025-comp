package frc.robot.lights;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Optional;

public enum LightsState {
  ERROR(Color.kRed, 0.08),
  UNHOMED(Color.kOrangeRed, 0.25),
  HOMED_NO_TAGS(Color.kYellow, 0.0),
  HOMED_SEES_TAGS(Color.kGreen, 0.0),

  BLINK(Color.kWhite, 0.08),

  INTAKING_CORAL(Color.kWhite, 0.25),
  INTAKING_ALGAE(Color.kTeal, 0.25),

  IDLE_EMPTY(Color.kBlack, 0.0),
  HOLDING_CORAL(Color.kWhite, 0.0),
  HOLDING_ALGAE(Color.kTeal, 0.0),

  LOLLIPOP_SEES_ALGAE(Color.kTeal, 0.0),
  LOLLIPOP_NO_ALGAE(Color.kBlack, 0.0),

  CORAL_HANDOFF(Color.kWhite, 0.08),

  CLIMB_LINEUP(Color.kYellow, 0.0),
  CLIMB_HANG(Color.kGreen, 0.0),
  CLIMB_STOP(Color.kGreen, 0.25),

  SCORE_NO_ALIGN_NO_TAGS(Color.kYellow, 0.25),
  SCORE_NO_ALIGN_TAGS(Color.kGreen, 0.25),
  SCORE_ALIGN_NO_TAGS(Color.kYellow, 0.08),
  SCORE_ALIGN_TAGS(Color.kGreen, 0.08),

  SCORING_ALGAE(Color.kTeal, 0.08),
  SCORING_CORAL(Color.kWhite, 0.08),

  OTHER(Color.kPurple, 0.25),

  /**
   * @deprecated Replace placeholder lights with actual light patterns.
   */
  @Deprecated
  PLACEHOLDER(Color.kBlack, 0.0);

  public final Optional<StrobeAnimation> stateBlinkRequest;
  public final Optional<SolidColor> stateColorRequest;

  public final Color color;
  public final double duration;

  LightsState(Color color, double duration) {
    this.color = color;
    this.duration = duration;
    if (duration == 0.0) {
      this.stateBlinkRequest = Optional.empty();
      this.stateColorRequest =
          Optional.of(
              new SolidColor(0, 399).withColor(new RGBWColor(color)).withUpdateFreqHz(50.0));
    } else {
      this.stateBlinkRequest =
          Optional.of(
              new StrobeAnimation(0, 399)
                  .withColor(new RGBWColor(color))
                  .withFrameRate(1 / duration)
                  .withUpdateFreqHz(50.0));
      this.stateColorRequest = Optional.empty();
    }
  }

  public boolean blinks() {
    return stateBlinkRequest.isPresent();
  }

  public ControlRequest getControlRequest() {
    return blinks() ? stateBlinkRequest.orElseThrow() : stateColorRequest.orElseThrow();
  }
}
