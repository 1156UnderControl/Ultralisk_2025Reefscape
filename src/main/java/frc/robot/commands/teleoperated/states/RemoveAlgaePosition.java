package frc.robot.commands.teleoperated.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.teleoperated.scorer.MoveScorerToRemovePosition;
import frc.robot.joysticks.ControlBoard;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RemoveAlgaePosition extends SequentialCommandGroup {
  ControlBoard controlBoard = ControlBoard.getInstance();

  public RemoveAlgaePosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new MoveScorerToRemovePosition(superStructure),
        Commands.idle(superStructure));
  }
}
