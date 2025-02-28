package frc.robot.commands.autonomous.auto_states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.teleoperated.intake.CollectCoralFromHP;
import frc.robot.commands.teleoperated.scorer.MoveScorerToCollectPosition;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoCollectCoral extends SequentialCommandGroup {
  public AutoCollectCoral(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new MoveScorerToCollectPosition(superStructure),
        new CollectCoralFromHP(superStructure),
        Commands.waitSeconds(0.2));
  }
}
