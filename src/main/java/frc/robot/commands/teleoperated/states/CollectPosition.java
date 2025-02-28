package frc.robot.commands.teleoperated.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.teleoperated.intake.CollectCoralFromHP;
import frc.robot.commands.teleoperated.scorer.MoveScorerToCollectPosition;
import frc.robot.commands.teleoperated.swerve.SwerveAngleWithCoralStation;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class CollectPosition extends SequentialCommandGroup {

  public CollectPosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new MoveScorerToCollectPosition(superStructure),
        Commands.race(new CollectCoralFromHP(superStructure), new SwerveAngleWithCoralStation(swerve)),
        Commands.waitSeconds(0.2));
  }
}
