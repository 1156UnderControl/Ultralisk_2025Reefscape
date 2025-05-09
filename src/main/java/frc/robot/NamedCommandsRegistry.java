package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.robot.commands.autonomous_commands.AutoGoToPoseCollectAlgaeFromReef;
import frc.robot.commands.autonomous_commands.AutoPrepareToRemoveAlgae;
import frc.robot.commands.autonomous_commands.AutoPrepareToScoreAlgaeNet;
import frc.robot.commands.autonomous_commands.AutoScoreAlgae;
import frc.robot.commands.autonomous_commands.AutoScoreAndPrepareToRemoveAlgaeFromBranch;
import frc.robot.commands.autonomous_commands.AutoScoreCoralAutonomous;
import frc.robot.commands.autonomous_commands.AutoScoreCoralAutonomousOptimized;
import frc.robot.commands.autonomous_commands.AutoScoreCoralAutonomousOptimizedDirect;
import frc.robot.commands.autonomous_commands.AutoScoreCoralAutonomousOptimizedDirectWithoutBackup;
import frc.robot.commands.autonomous_commands.AutoUpdateOdometry;
import frc.robot.commands.autonomous_commands.CollectAutonomous;
import frc.robot.commands.autonomous_commands.CollectAutonomousOpitimized;
import frc.robot.commands.scorer.MoveScorerToRemovePosition;
import frc.robot.commands.states.DefaultPosition;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class NamedCommandsRegistry {
  SuperStructure superStructure;
  SwerveSubsystem drivetrain;

  public NamedCommandsRegistry(SwerveSubsystem drivetrain, SuperStructure superStructure) {
    this.superStructure = superStructure;
    this.drivetrain = drivetrain;
  }

  public void registerAllAutoCommands() {
    this.registerResetOdometryCommands(this.drivetrain);
    this.registerIntakeCommands(this.superStructure);
    this.registerCoralScoringCommands(this.superStructure, this.drivetrain);
    this.registerAlgaeCommands(this.superStructure);
    this.registerAlgaeRemovalCommands(this.superStructure, this.drivetrain);
    this.registerLevelCommands(this.superStructure);
  }

  private void registerResetOdometryCommands(SwerveSubsystem drivetrain) {
    NamedCommands.registerCommand("ResetOdometry Left",
        new InstantCommand(() -> drivetrain.resetOdometry(
            AllianceFlipUtil.apply(new Pose2d(7.18, 4.730, Rotation2d.k180deg)))));
    NamedCommands.registerCommand("ResetOdometry Right",
        new InstantCommand(() -> drivetrain.resetOdometry(
            AllianceFlipUtil.apply(new Pose2d(7.18, 3.317, Rotation2d.k180deg)))));

    NamedCommands.registerCommand("ResetOdometry Left XY",
        new InstantCommand(() -> drivetrain.resetTranslation(
            AllianceFlipUtil.apply(new Translation2d(7.18, 4.730)))));
    NamedCommands.registerCommand("ResetOdometry Right XY",
        new InstantCommand(() -> drivetrain.resetTranslation(
            AllianceFlipUtil.apply(new Translation2d(7.18, 3.317)))));

    NamedCommands.registerCommand("ResetOdometry Center XY With Vision",
        new AutoUpdateOdometry(drivetrain, new Translation2d(7.18, 4)));
    NamedCommands.registerCommand("ResetOdometry Right XY With Vision",
        new AutoUpdateOdometry(drivetrain, new Translation2d(7.18, 3.317)));
    NamedCommands.registerCommand("ResetOdometry Left XY With Vision",
        new AutoUpdateOdometry(drivetrain, new Translation2d(7.18, 4.730)));
  }

  private void registerIntakeCommands(SuperStructure superStructure) {
    NamedCommands.registerCommand("Intake Coral Optimized",
        new CollectAutonomousOpitimized(superStructure));
    NamedCommands.registerCommand("Intake Coral",
        new CollectAutonomous(superStructure)
            .andThen(new DefaultPosition(superStructure))
            .finallyDo(() -> superStructure.scorer.stopEndEffector()));
  }

  private void registerCoralScoringCommands(SuperStructure superStructure, SwerveSubsystem drivetrain) {
    for (TargetBranch branch : TargetBranch.values()) {
      String name = branch.name();
      NamedCommands.registerCommand("Score Coral " + name,
          new AutoScoreCoralAutonomous(superStructure, drivetrain, branch));
      NamedCommands.registerCommand("Score Coral " + name + " Optimized",
          new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, branch));
      NamedCommands.registerCommand("Score Coral " + name + " Optimized Direct",
          new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, branch));
      NamedCommands.registerCommand("Score Coral " + name + " Optimized Direct Without Backup",
          new AutoScoreCoralAutonomousOptimizedDirectWithoutBackup(superStructure, drivetrain, branch));
      NamedCommands.registerCommand("Score Coral " + name + " Optimized Direct Without Backup And Remove Algae",
          new AutoScoreAndPrepareToRemoveAlgaeFromBranch(superStructure, drivetrain, branch));
    }
  }

  private void registerAlgaeCommands(SuperStructure superStructure) {
    NamedCommands.registerCommand("Prepare To Score Algae On Net",
        new AutoPrepareToScoreAlgaeNet(superStructure));
    NamedCommands.registerCommand("Score Algae",
        new AutoScoreAlgae(superStructure));
  }

  private void registerAlgaeRemovalCommands(SuperStructure superStructure, SwerveSubsystem drivetrain) {
    NamedCommands.registerCommand("Prepare To Remove Algae From A-B",
        new AutoPrepareToRemoveAlgae(superStructure, TargetBranch.A));
    NamedCommands.registerCommand("Prepare To Remove Algae From C-D",
        new AutoPrepareToRemoveAlgae(superStructure, TargetBranch.C));
    NamedCommands.registerCommand("Prepare To Remove Algae From E-F",
        new AutoPrepareToRemoveAlgae(superStructure, TargetBranch.E));
    NamedCommands.registerCommand("Prepare To Remove Algae From G-H",
        new AutoPrepareToRemoveAlgae(superStructure, TargetBranch.G));
    NamedCommands.registerCommand("Prepare To Remove Algae From I-J",
        new AutoPrepareToRemoveAlgae(superStructure, TargetBranch.I));
    NamedCommands.registerCommand("Prepare To Remove Algae From K-L",
        new AutoPrepareToRemoveAlgae(superStructure, TargetBranch.K));

    NamedCommands.registerCommand("Remove Algae From A-B",
        new AutoGoToPoseCollectAlgaeFromReef(drivetrain, superStructure, TargetBranch.A)
            .alongWith(new MoveScorerToRemovePosition(superStructure)));
    NamedCommands.registerCommand("Remove Algae From C-D",
        new AutoGoToPoseCollectAlgaeFromReef(drivetrain, superStructure, TargetBranch.C)
            .alongWith(new MoveScorerToRemovePosition(superStructure)));
    NamedCommands.registerCommand("Remove Algae From E-F",
        new AutoGoToPoseCollectAlgaeFromReef(drivetrain, superStructure, TargetBranch.E)
            .alongWith(new MoveScorerToRemovePosition(superStructure)));
    NamedCommands.registerCommand("Remove Algae From G-H",
        new AutoGoToPoseCollectAlgaeFromReef(drivetrain, superStructure, TargetBranch.G)
            .alongWith(new MoveScorerToRemovePosition(superStructure)));
    NamedCommands.registerCommand("Remove Algae From I-J",
        new AutoGoToPoseCollectAlgaeFromReef(drivetrain, superStructure, TargetBranch.I)
            .alongWith(new MoveScorerToRemovePosition(superStructure)));
    NamedCommands.registerCommand("Remove Algae From K-L",
        new AutoGoToPoseCollectAlgaeFromReef(drivetrain, superStructure, TargetBranch.K)
            .alongWith(new MoveScorerToRemovePosition(superStructure)));
  }

  private void registerLevelCommands(SuperStructure superStructure) {
    for (ReefLevel level : ReefLevel.values()) {
      if (level != ReefLevel.TO_L4) {
        NamedCommands.registerCommand("Set Coral Level " + level.name().replace("L", ""),
            new InstantCommand(() -> superStructure.scorer.setTargetBranchLevel(level)));
      }
    }
  }
}
