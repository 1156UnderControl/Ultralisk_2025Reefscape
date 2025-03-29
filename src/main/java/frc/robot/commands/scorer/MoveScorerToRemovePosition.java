package frc.robot.commands.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.Java_Is_UnderControl.LEDs.LedSubsystem;
import frc.robot.SuperStructure;

public class MoveScorerToRemovePosition extends Command {
  SuperStructure superStructure;
  LedSubsystem led = LedSubsystem.getInstance();

  public MoveScorerToRemovePosition(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.scorer.removeAlgaeFromBranch();
    superStructure.led.setSolidColor(LedColor.RED);
  }

  @Override
  public void execute() {
    superStructure.scorer.removeAlgaeFromBranch();
    if (superStructure.scorer.isAtRemovePosition()) {
      superStructure.scorer.removeAlgaeEndEffector();
    } else {
      superStructure.scorer.stopEndEffector();
    }
  }

  @Override
  public void end(boolean interrupted) {
    superStructure.led.setSolidColor(LedColor.GREEN);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
