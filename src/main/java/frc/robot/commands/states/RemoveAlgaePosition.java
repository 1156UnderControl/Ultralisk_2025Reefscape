package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Java_Is_UnderControl.Vision.Deprecated.Cameras.LimelightHelpers;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToRemovePosition;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.joysticks.ControlBoard;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RemoveAlgaePosition extends SequentialCommandGroup {
  ControlBoard controlBoard = ControlBoard.getInstance();

  public RemoveAlgaePosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new MoveScorerToRemovePosition(superStructure),
        new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceBlink("limelight-right")),
        Commands.idle(superStructure).onlyIf(() -> superStructure.scorer.getTargetReefLevel() != ReefLevel.L1));
  }
}
