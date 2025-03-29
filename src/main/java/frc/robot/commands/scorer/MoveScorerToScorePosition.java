package frc.robot.commands.scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Java_Is_UnderControl.LEDs.LedColor;
import frc.Java_Is_UnderControl.LEDs.LedSubsystem;
import frc.robot.SuperStructure;
import frc.robot.joysticks.OperatorController;

public class MoveScorerToScorePosition extends Command {
  SuperStructure superStructure;
  OperatorController operatorController = OperatorController.getInstance();
  LedSubsystem led = LedSubsystem.getInstance();

  public MoveScorerToScorePosition(SuperStructure superStructure) {
    this.superStructure = superStructure;
    addRequirements(this.superStructure);
  }

  @Override
  public void initialize() {
    superStructure.intake.stopIntake();
    superStructure.scorer.prepareToPlaceCoralOnBranch();
    superStructure.led.setSolidColor(LedColor.RED);
  }

  @Override
  public void execute() {
    superStructure.scorer.prepareToPlaceCoralOnBranch();
  }

  @Override
  public void end(boolean interrupted) {
    superStructure.led.setSolidColor(LedColor.GREEN);
  }

  @Override
  public boolean isFinished() {
    return superStructure.scorer.isSecuredToPlaceCoral();
  }
}
