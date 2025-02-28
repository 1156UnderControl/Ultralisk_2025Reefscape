package frc.robot.commands.autonomous.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructure;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.joysticks.OperatorController;

public class MoveScorerToScorePosition extends Command {
  SuperStructure superStructure;
  ReefLevel reefLevel;
  OperatorController operatorController = OperatorController.getInstance();

  public MoveScorerToScorePosition(SuperStructure superStructure, ReefLevel reefLevel) {
    this.superStructure = superStructure;
    this.reefLevel = reefLevel;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.intake.stopIntake();
    superStructure.scorer.prepareToPlaceCoralOnBranch();
  }

  @Override
  public void execute() {
    superStructure.scorer.prepareToPlaceCoralOnBranch();
  }

  @Override
  public boolean isFinished() {
    return operatorController.scoreCoral().getAsBoolean() && superStructure.scorer.isSecuredToPlaceCoral();
  }
}
