package frc.robot.commands.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.Java_Is_UnderControl.LEDs.LedSubsystem;
import frc.robot.SuperStructure;

public class MoveScorerToDefaultPosition extends Command {
  SuperStructure superStructure;
  LedSubsystem led = LedSubsystem.getInstance();

  public MoveScorerToDefaultPosition(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.scorer.moveScorerToDefaultPosition();
  }

  @Override
  public void execute() {
    superStructure.scorer.moveScorerToDefaultPosition();
  }

  @Override
  public void end(boolean interrupted) {
    led.setSolidColor(LedColor.GREEN);
  }

  @Override
  public boolean isFinished() {
    return superStructure.scorer.isAtDefaultPosition();
  }
}
