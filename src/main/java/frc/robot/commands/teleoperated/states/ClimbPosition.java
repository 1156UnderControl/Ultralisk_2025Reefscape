package frc.robot.commands.teleoperated.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.teleoperated.climber.MoveClimberToClimbedPosition;
import frc.robot.commands.teleoperated.climber.MoveClimberToRaisedPosition;
import frc.robot.commands.teleoperated.scorer.MoveScorerToDefaultPosition;
import frc.robot.joysticks.ControlBoard;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ClimbPosition extends SequentialCommandGroup {
  ControlBoard controlBoard = ControlBoard.getInstance();

  public ClimbPosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new MoveScorerToDefaultPosition(superStructure),
        new MoveClimberToRaisedPosition(superStructure),
        Commands.waitSeconds(1.5),
        new MoveClimberToClimbedPosition(superStructure),
        Commands.idle(superStructure));
  }
}
