package frc.robot.commands.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.robot.SuperStructure;

public class MoveScorerToCollectPosition extends Command {
  SuperStructure superStructure;

  public MoveScorerToCollectPosition(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.scorer.intakeFromHP();
  }

  @Override
  public void execute() {
    if (superStructure.scorer.isAtCollectPosition()) {
      superStructure.led.setSolidColor(LedColor.GREEN);
    } else {
      superStructure.led.setSolidColor(LedColor.RED);

    }
    superStructure.scorer.intakeFromHP();
  }

  @Override
  public boolean isFinished() {
    return superStructure.scorer.hasCoral();
  }
}
