package frc.robot.autos;

import frc.robot.autos.auto_path_commands.blue.BlueDoNothingAuto;
import frc.robot.autos.auto_path_commands.red.RedDoNothingAuto;
import frc.robot.autos.auto_path_commands.red.RedHybrid4PieceAuto;
import frc.robot.autos.auto_path_commands.red.RedLollipopAuto;
import frc.robot.autos.auto_path_commands.red.RedLollipopDescoreKL;
import frc.robot.autos.auto_path_commands.red.RedLollipopL2AB;
import frc.robot.autos.auto_path_commands.red.RedLollipopVariantAuto;
import frc.robot.autos.auto_path_commands.red.RedStraightLineAuto;
import frc.robot.autos.auto_path_commands.red.RedThreePiece2ILKAuto;
import frc.robot.autos.auto_path_commands.red.RedThreePiece3GHJ;
import frc.robot.autos.auto_path_commands.red.RedThreePiece3IKLAuto;
import frc.robot.robot_manager.RobotManager;
import java.util.function.BiFunction;

public enum AutoSelection {
  DO_NOTHING(RedDoNothingAuto::new, BlueDoNothingAuto::new),
  STRAIGHT_LINE(RedStraightLineAuto::new, RedStraightLineAuto::new),
  THREE_PIECE_2ILK(RedThreePiece2ILKAuto::new, RedThreePiece2ILKAuto::new),
  THREE_PIECE_3IKL(RedThreePiece3IKLAuto::new, RedThreePiece3IKLAuto::new),
  LOLLIPOP(RedLollipopAuto::new, RedLollipopAuto::new),
  LOLLIPOP_L3_DESCRORE_KL(RedLollipopDescoreKL::new, RedLollipopDescoreKL::new),
  LOLLIPOP_L2_AB(RedLollipopL2AB::new, RedLollipopL2AB::new),

  LOLLIPOP_VARIANT(RedLollipopVariantAuto::new, RedLollipopVariantAuto::new),
  THREE_PIECE_3GHJ(RedThreePiece3GHJ::new, RedThreePiece3GHJ::new),
  HYBRID_AUTO(RedHybrid4PieceAuto::new, RedHybrid4PieceAuto::new);

  public final BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto;
  public final BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto;

  private AutoSelection(
      BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto,
      BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto) {
    this.redAuto = redAuto;
    this.blueAuto = blueAuto;
  }
}
