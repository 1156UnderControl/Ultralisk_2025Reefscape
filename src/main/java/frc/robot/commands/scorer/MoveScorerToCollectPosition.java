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
    superStructure.scorer.intakeFromHP();
  }

  @Override
  public boolean isFinished() {
    superStructure.led.setSolidColor(LedColor.RED);
    return superStructure.scorer.hasCoral();
  }
}
