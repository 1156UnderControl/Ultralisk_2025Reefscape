package frc.robot.commands.teleoperated.teleop_states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.teleoperated.climber.IntakeCageClimber;
import frc.robot.commands.teleoperated.climber.MoveClimberToClimbedPosition;
import frc.robot.commands.teleoperated.climber.MoveClimberToIntakePosition;
import frc.robot.commands.teleoperated.scorer.MoveScorerToDefaultPositionWithCoral;
import frc.robot.joysticks.OperatorController;

public class ClimbPosition extends SequentialCommandGroup {
  OperatorController operatorKeyboard = OperatorController.getInstance();

  public ClimbPosition(SuperStructure superStructure) {
    addCommands(new MoveScorerToDefaultPositionWithCoral(superStructure),
        new MoveClimberToIntakePosition(superStructure),
        new IntakeCageClimber(superStructure),
        Commands.waitUntil(operatorKeyboard.climb()),
        new MoveClimberToClimbedPosition(superStructure),
        new InstantCommand(() -> superStructure.robotIsClimbed = true),
        Commands.idle(superStructure));
  }
}
