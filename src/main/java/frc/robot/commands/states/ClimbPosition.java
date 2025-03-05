package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.climber.IntakeCageClimber;
import frc.robot.commands.climber.MoveClimberToClimbedPosition;
import frc.robot.commands.climber.MoveClimberToIntakePosition;
import frc.robot.commands.climber.StopClimberMotor;
import frc.robot.commands.scorer.MoveScorerToDefaultPosition;
import frc.robot.joysticks.OperatorController;

public class ClimbPosition extends SequentialCommandGroup {
  OperatorController operatorKeyboard = OperatorController.getInstance();

  public ClimbPosition(SuperStructure superStructure) {
    addCommands(new MoveScorerToDefaultPosition(superStructure),
        new MoveClimberToIntakePosition(superStructure),
        new IntakeCageClimber(superStructure),
        Commands.waitUntil(operatorKeyboard.climb()),
        new MoveClimberToClimbedPosition(superStructure),
        new StopClimberMotor(superStructure),
        new InstantCommand(() -> superStructure.robotIsClimbed = true),
        Commands.idle(superStructure));
  }
}
