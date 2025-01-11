package frc.robot.autos.trailblazer;

import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection {
  DO_NOTHING(DoNothingAuto::new),
  THREE_PIECE_2IJK(Auto3Piece2IJK::new),
  PUSH_PARTNER(PushPartnerAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseAuto> auto;

  private AutoSelection(BiFunction<RobotManager, Trailblazer, BaseAuto> auto) {
    this.auto = auto;
  }
}
