package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.intake.CollectCoralFromHP;
import frc.robot.commands.scorer.MoveScorerToCollectPosition;
import frc.robot.commands.swerve.SwerveAlignWithCoralStation;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class CollectPosition extends SequentialCommandGroup {

  public CollectPosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new MoveScorerToCollectPosition(superStructure),
        Commands.race(new CollectCoralFromHP(superStructure), new SwerveAlignWithCoralStation(swerve)));
  }
}
