package frc.robot.lights;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.team581.util.state_machines.StateMachine;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.scheduling.SubsystemPriority;

public class LightsSubsystem extends StateMachine<LightsState> {
  private final CANdle candle;

  private final Timer blinkTimer = new Timer();
  private LightsState storedState = LightsState.IDLE_EMPTY;
  private LightsState disabledState = LightsState.HOMED_SEES_TAGS;

  private final double BLINK_FAST_FRAMERATE = 500.0; //The frame rate of the animation, from [1, 500] Hz.
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
private void setControlSolid(RGBWColor color){
  candle.setControl(new SolidColor(0, 0).withColor(color));
}
private void setControlStrobed(RGBWColor color, double frameRate){
  candle.setControl(new StrobeAnimation(0, 0).withColor(color).withFrameRate(frameRate));
}
  @Override
  protected void afterTransition(LightsState newState) {
      switch (newState) {
        case BLINK -> setControlStrobed(new RGBWColor(Color.kWhite), BLINK_FAST_FRAMERATE);
        case ERROR -> setControlStrobed(new RGBWColor(Color.kRed), BLINK_FAST_FRAMERATE);
        case UNHOMED -> setControlStrobed(new RGBWColor(Color.kWhite), BLINK_SLOW_FRAMERATE);
        case HOMED_NO_TAGS -> setControlSolid(new RGBWColor(Color.kYellow));
        case HOMED_SEES_TAGS -> setControlSolid(new RGBWColor(Color.kGreen));

        case INTAKING_CORAL -> setControlStrobed(new RGBWColor(Color.kWhite), BLINK_SLOW_FRAMERATE);
        case INTAKING_ALGAE -> setControlStrobed(new RGBWColor(Color.kTeal), BLINK_SLOW_FRAMERATE);

        case IDLE_EMPTY -> setControlSolid(new RGBWColor(Color.kBlack));
        case HOLDING_CORAL -> setControlSolid(new RGBWColor(Color.kWhite));
        case HOLDING_ALGAE -> setControlSolid(new RGBWColor(Color.kTeal));

        case LOLLIPOP_SEES_ALGAE -> setControlSolid(new RGBWColor(Color.kTeal));
        case LOLLIPOP_NO_ALGAE -> setControlSolid(new RGBWColor(Color.kBlack));

        case CORAL_HANDOFF -> setControlStrobed(new RGBWColor(Color.kWhite), BLINK_FAST_FRAMERATE);

        case CLIMB_LINEUP -> setControlSolid(new RGBWColor(Color.kYellow));
        case CLIMB_HANG -> setControlSolid(new RGBWColor(Color.kGreen));
        case CLIMB_STOP -> setControlStrobed(new RGBWColor(Color.kGreen), BLINK_SLOW_FRAMERATE);

        case SCORE_NO_ALIGN_NO_TAGS -> setControlStrobed(new RGBWColor(Color.kYellow), BLINK_SLOW_FRAMERATE);
        case SCORE_NO_ALIGN_TAGS -> setControlStrobed(new RGBWColor(Color.kGreen), BLINK_SLOW_FRAMERATE);
        case SCORE_ALIGN_NO_TAGS -> setControlStrobed(new RGBWColor(Color.kYellow), BLINK_FAST_FRAMERATE);
        case SCORE_ALIGN_TAGS -> setControlStrobed(new RGBWColor(Color.kGreen), BLINK_FAST_FRAMERATE);

        case SCORING_ALGAE -> setControlStrobed(new RGBWColor(Color.kTeal), BLINK_FAST_FRAMERATE);
        case SCORING_CORAL -> setControlStrobed(new RGBWColor(Color.kWhite), BLINK_FAST_FRAMERATE);

        case OTHER -> setControlStrobed(new RGBWColor(Color.kPurple), BLINK_SLOW_FRAMERATE);
        default -> setControlSolid(new RGBWColor(Color.kBlack));
  }
}}
