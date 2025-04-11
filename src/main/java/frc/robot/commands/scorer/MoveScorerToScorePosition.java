package frc.robot.commands.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.robot.SuperStructure;
import frc.robot.joysticks.OperatorController;

public class MoveScorerToScorePosition extends Command {
  SuperStructure superStructure;
  OperatorController operatorController = OperatorController.getInstance();

  public MoveScorerToScorePosition(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    superStructure.led.setIntakeLeftHeightColor(LedColor.YELLOW);
    superStructure.led.setIntakeRightHeightColor(LedColor.YELLOW);
    superStructure.led.setElevatorLeftHeightColor(LedColor.YELLOW);
    superStructure.led.setElevatorRightHeightColor(LedColor.YELLOW);

    if (this.superStructure.scorer.hasCoral()) {
      superStructure.scorer.prepareToPlaceCoralOnBranch();
    }
    if (this.superStructure.scorer.hasAlgae()) {
      superStructure.scorer.prepareToScoreAlgae();
    }
  }

  @Override
  public boolean isFinished() {
    if (this.superStructure.scorer.hasCoral()) {
      return superStructure.scorer.isSecuredToPlaceCoral();
    }
    if (this.superStructure.scorer.hasAlgae()) {
      return superStructure.scorer.isSecuredToScoreOnNet();
    }
    return true;
  }
}
