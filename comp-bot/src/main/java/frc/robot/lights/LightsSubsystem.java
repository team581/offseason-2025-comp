package frc.robot.lights;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.scheduling.SubsystemPriority;

public class LightsSubsystem extends StateMachine<LightsState> {
  private final CANdle candle;

  private final Timer blinkTimer = new Timer();
  private LightsState storedState = LightsState.IDLE_EMPTY;
  private LightsState disabledState = LightsState.HOMED_SEES_TAGS;

  private final double BLINK_FAST_FRAMERATE = 500.0;
  private final double BLINK_SLOW_FRAMERATE = 50.0;

  public LightsSubsystem(CANdle candle) {
    super(SubsystemPriority.LIGHTS, LightsState.IDLE_EMPTY);
    this.candle = candle;
    blinkTimer.start();
  }

  public void setState(LightsState newState) {
    setStateFromRequest(newState);
  }

  public void blink() {
    storedState = getState();
    setStateFromRequest(LightsState.BLINK);
  }

  public void setDisabledState(LightsState newDisabledState) {
    disabledState = newDisabledState;
  }

  @Override
  protected LightsState getNextState(LightsState currentState) {
    return switch (currentState) {
      case BLINK -> timeout(1.0) ? storedState : currentState;
      default -> currentState;
    };
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
  }

  @Override
  protected void afterTransition(LightsState newState) {
      switch (newState) {
        case BLINK -> candle.setControl(new StrobeAnimation(0, 0).withColor(new RGBWColor(Color.kWhite)).withFrameRate(BLINK_FAST_FRAMERATE));

        case ERROR -> candle.setControl(new StrobeAnimation(0, 0).withColor(new RGBWColor(Color.kRed)).withFrameRate(BLINK_FAST_FRAMERATE));
        case UNHOMED -> candle.setControl(new StrobeAnimation(0, 0).withColor(new RGBWColor(Color.kWhite)).withFrameRate(BLINK_SLOW_FRAMERATE));
        case HOMED_NO_TAGS -> candle.setControl(new SolidColor(0, 0).withColor(new RGBWColor(Color.kYellow)));
        case HOMED_SEES_TAGS -> candle.setControl(new SolidColor(0, 0).withColor(new RGBWColor(Color.kGreen)));

        case INTAKING_CORAL -> candle.setControl(new StrobeAnimation(0, 0).withColor(new RGBWColor(Color.kWhite)).withFrameRate(BLINK_SLOW_FRAMERATE));
        case INTAKING_ALGAE -> candle.setControl(new StrobeAnimation(0, 0).withColor(new RGBWColor(Color.kTeal)).withFrameRate(BLINK_SLOW_FRAMERATE));

        case IDLE_EMPTY -> candle.setControl(new SolidColor(0, 0).withColor(new RGBWColor(Color.kBlack)));
        case HOLDING_CORAL -> candle.setControl(new SolidColor(0, 0).withColor(new RGBWColor(Color.kWhite)));
        case HOLDING_ALGAE -> candle.setControl(new SolidColor(0, 0).withColor(new RGBWColor(Color.kTeal)));

        case LOLLIPOP_SEES_ALGAE -> candle.setControl(new SolidColor(0, 0).withColor(new RGBWColor(Color.kTeal)));
        case LOLLIPOP_NO_ALGAE -> candle.setControl(new SolidColor(0, 0).withColor(new RGBWColor(Color.kBlack)));

        case CORAL_HANDOFF -> candle.setControl(new StrobeAnimation(0, 0).withColor(new RGBWColor(Color.kWhite)).withFrameRate(BLINK_FAST_FRAMERATE));

        case CLIMB_LINEUP -> candle.setControl(new SolidColor(0, 0).withColor(new RGBWColor(Color.kYellow)));
        case CLIMB_HANG -> candle.setControl(new SolidColor(0, 0).withColor(new RGBWColor(Color.kGreen)));
        case CLIMB_STOP -> candle.setControl(new StrobeAnimation(0, 0).withColor(new RGBWColor(Color.kGreen)).withFrameRate(BLINK_SLOW_FRAMERATE));

        case SCORE_NO_ALIGN_NO_TAGS -> candle.setControl(new StrobeAnimation(0, 0).withColor(new RGBWColor(Color.kYellow)).withFrameRate(BLINK_SLOW_FRAMERATE));
        case SCORE_NO_ALIGN_TAGS -> candle.setControl(new StrobeAnimation(0, 0).withColor(new RGBWColor(Color.kGreen)).withFrameRate(BLINK_SLOW_FRAMERATE));
        case SCORE_ALIGN_NO_TAGS -> candle.setControl(new StrobeAnimation(0, 0).withColor(new RGBWColor(Color.kYellow)).withFrameRate(BLINK_FAST_FRAMERATE));
        case SCORE_ALIGN_TAGS -> candle.setControl(new StrobeAnimation(0, 0).withColor(new RGBWColor(Color.kGreen)).withFrameRate(BLINK_FAST_FRAMERATE));

        case SCORING_ALGAE -> candle.setControl(new StrobeAnimation(0, 0).withColor(new RGBWColor(Color.kTeal)).withFrameRate(BLINK_FAST_FRAMERATE));
        case SCORING_CORAL -> candle.setControl(new StrobeAnimation(0, 0).withColor(new RGBWColor(Color.kWhite)).withFrameRate(BLINK_FAST_FRAMERATE));

        case OTHER -> candle.setControl(new StrobeAnimation(0, 0).withColor(new RGBWColor(Color.kPurple)).withFrameRate(BLINK_SLOW_FRAMERATE));















  }
}}
