package frc.robot.autos.trailblazer;

import java.util.function.BiFunction;

import frc.robot.robot_manager.RobotManager;

public enum AutoSelection {
  DO_NOTHING(DoNothingAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseAuto> auto;

  private AutoSelection(BiFunction<RobotManager, Trailblazer, BaseAuto> auto) {
    this.auto = auto;
  }
}
