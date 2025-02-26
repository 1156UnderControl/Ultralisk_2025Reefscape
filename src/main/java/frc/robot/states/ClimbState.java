package frc.robot.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.climber.MoveClimberToClimbedPosition;
import frc.robot.commands.climber.MoveClimberToRaisedPosition;
import frc.robot.commands.scorer.MoveScorerToDefaultPosition;
import frc.robot.joysticks.ControlBoard;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ClimbState extends SequentialCommandGroup {
  ControlBoard controlBoard = ControlBoard.getInstance();

  public ClimbState(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new MoveScorerToDefaultPosition(superStructure),
        new MoveClimberToRaisedPosition(superStructure),
        new MoveClimberToClimbedPosition(superStructure),
        Commands.idle(superStructure));
  }
}
