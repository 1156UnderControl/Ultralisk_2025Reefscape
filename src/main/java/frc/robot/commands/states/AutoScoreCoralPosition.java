package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SuperStructure;
import frc.robot.commands.scorer.MoveScorerToScorePosition;
import frc.robot.commands.swerve.SwerveGoToBranchBackup;
import frc.robot.commands.util.GoAndRaiseElevator;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.IDriverController;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoScoreCoralPosition extends SequentialCommandGroup {
  OperatorController operatorKeyboard = OperatorController.getInstance();
  IDriverController driverController = DriverController.getInstance();
  boolean hasCancelledAutoMove = false;

  public AutoScoreCoralPosition(SuperStructure superStructure, SwerveSubsystem swerve, TargetBranch branch) {
    addCommands(new InstantCommand(() -> hasCancelledAutoMove = false),
        Commands.either(
            new SwerveGoToBranchBackup(swerve, branch).andThen(new MoveScorerToScorePosition(superStructure)),
            Commands.none(),
            () -> swerve.checkPivotWillCrashOnReef()),
        new GoAndRaiseElevator(swerve, superStructure, branch, driverController.isForcingDriverControl().getAsBoolean())
            .until(() -> {
              if (operatorKeyboard.scoreCoral().getAsBoolean()) {
                hasCancelledAutoMove = true;
              }
              return hasCancelledAutoMove;
            }),
        new ParallelRaceGroup(Commands.idle(superStructure)
            .until(operatorKeyboard.scoreCoral()
                .or(() -> swerve.isAtTargetPositionWithoutHeading() && superStructure.scorer.isScorerAtPosition())
                .or(() -> hasCancelledAutoMove)),
            Commands.run(() -> swerve.driveAlignAngleJoystick(), swerve)),
        Commands.run(() -> superStructure.scorer.placeCoral())
            .alongWith(Commands.run(() -> swerve.driveAlignAngleJoystick(), swerve)));
  }
}
