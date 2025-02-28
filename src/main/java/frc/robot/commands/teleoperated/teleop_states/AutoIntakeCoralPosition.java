package frc.robot.commands.teleoperated.teleop_states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.teleoperated.intake.CollectCoralFromHP;
import frc.robot.commands.teleoperated.scorer.MoveScorerToCollectPosition;
import frc.robot.commands.teleoperated.swerve.SwerveAlignWithCoralStation;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoIntakeCoralPosition extends SequentialCommandGroup {

  public AutoIntakeCoralPosition(SuperStructure superStructure, SwerveSubsystem swerve) {
    addCommands(new MoveScorerToCollectPosition(superStructure),
        Commands.race(new CollectCoralFromHP(superStructure), new SwerveAlignWithCoralStation(swerve)),
        Commands.waitSeconds(0.2));
  }
}
