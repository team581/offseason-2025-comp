package frc.robot.autos;

import frc.robot.autos.auto_path_commands.blue.BlueDoNothingAuto;
import frc.robot.autos.auto_path_commands.blue.BlueLollipopLeftABAuto;
import frc.robot.autos.auto_path_commands.blue.BlueLollipopLeftL4Auto;
import frc.robot.autos.auto_path_commands.blue.BlueLollipopRightABAuto;
import frc.robot.autos.auto_path_commands.blue.BlueLollipopRightL4Auto;
import frc.robot.autos.auto_path_commands.blue.BlueStraightLineAuto;
import frc.robot.autos.auto_path_commands.red.RedDoNothingAuto;
import frc.robot.autos.auto_path_commands.red.RedLollipopLeftABAuto;
import frc.robot.autos.auto_path_commands.red.RedLollipopLeftL4Auto;
import frc.robot.autos.auto_path_commands.red.RedLollipopRightABAuto;
import frc.robot.autos.auto_path_commands.red.RedLollipopRightL4Auto;
import frc.robot.autos.auto_path_commands.red.RedStraightLineAuto;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.trailblazer.Trailblazer;
import java.util.function.BiFunction;

public enum AutoSelection {
  DO_NOTHING(RedDoNothingAuto::new, BlueDoNothingAuto::new),

  STRAIGHT_LINE(RedStraightLineAuto::new, BlueStraightLineAuto::new),

  LOLLIPOP_LEFT_AB(RedLollipopLeftABAuto::new, BlueLollipopLeftABAuto::new),
  LOLLIPOP_RIGHT_AB(RedLollipopRightABAuto::new, BlueLollipopRightABAuto::new),

  LOLLIPOP_LEFT_L4(RedLollipopLeftL4Auto::new, BlueLollipopLeftL4Auto::new),
  LOLLIPOP_RIGHT_L4(RedLollipopRightL4Auto::new, BlueLollipopRightL4Auto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto;
  public final BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto;

  private AutoSelection(
      BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto,
      BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto) {
    this.redAuto = redAuto;
    this.blueAuto = blueAuto;
  }
}
