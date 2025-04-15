// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.robot.commands.autonomous_commands.AutoScoreCoralAutonomous;
import frc.robot.commands.autonomous_commands.AutoScoreCoralAutonomousOptimized;
import frc.robot.commands.autonomous_commands.AutoScoreCoralAutonomousOptimizedDirect;
import frc.robot.commands.autonomous_commands.AutoScoreCoralAutonomousOptimizedDirectWithoutBackup;
import frc.robot.commands.autonomous_commands.AutoUpdateOdometry;
import frc.robot.commands.autonomous_commands.CollectAutonomous;
import frc.robot.commands.autonomous_commands.CollectAutonomousOpitimized;
import frc.robot.commands.autonomous_commands.DefaultPositionAutonomous;
import frc.robot.commands.scorer.ScoreProcessor;
import frc.robot.commands.states.AutoScoreCoralPosition;
import frc.robot.commands.states.ClimbPosition;
import frc.robot.commands.states.CollectPosition;
import frc.robot.commands.states.DefaultPosition;
import frc.robot.commands.states.RemoveAlgaePosition;
import frc.robot.commands.states.ScoreObjectPosition;
import frc.robot.commands.swerve.SwerveAngleWithCoralStation;
import frc.robot.constants.FieldConstants.Algae.AlgaeHeightReef;
import frc.robot.constants.FieldConstants.ReefLevel;
import frc.robot.constants.SwerveConstants.TargetBranch;
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private OperatorController keyBoard = OperatorController.getInstance();

  private DriverController driverController = DriverController.getInstance();

  public final SuperStructure superStructure = new SuperStructure();

  private SwerveModuleConstants[] modulesArray = TunerConstants.getModuleConstants();

  public final SwerveSubsystem drivetrain = new SwerveSubsystem(() -> superStructure.scorer.isElevatorInHighPosition(),
      () -> superStructure.scorer.getTargetReefLevel(), () -> superStructure.scorer.getTargetReefLevelAlgae(),
      TunerConstants.getSwerveDrivetrainConstants(),
      modulesArray[0], modulesArray[1], modulesArray[2], modulesArray[3]);

  private final Telemetry logger = new Telemetry(drivetrain.MaxSpeed);

  public RobotContainer() {
    setNamedCommandsForAuto();
    configureBindings();
    this.autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);
    superStructure
        .setDefaultCommand(new DefaultPosition(superStructure));
    drivetrain.setDefaultCommand(
        Commands.run(() -> drivetrain.driveAlignAngleJoystick(), drivetrain)
            .onlyIf(() -> DriverStation.isTeleopEnabled()));
  }

  private void setNamedCommandsForAuto() {
    NamedCommands.registerCommand("ResetOdometry Left",
        new InstantCommand(
            () -> drivetrain.resetOdometry(AllianceFlipUtil.apply(new Pose2d(7.18, 4.730, Rotation2d.k180deg)))));
    NamedCommands.registerCommand("ResetOdometry Right",
        new InstantCommand(
            () -> drivetrain.resetOdometry(AllianceFlipUtil.apply(new Pose2d(7.18, 3.317, Rotation2d.k180deg)))));

    NamedCommands.registerCommand("ResetOdometry Left XY",
        new InstantCommand(
            () -> drivetrain.resetTranslation(AllianceFlipUtil.apply(new Translation2d(7.18, 4.73)))));
    NamedCommands.registerCommand("ResetOdometry Right XY",
        new InstantCommand(
            () -> drivetrain.resetTranslation(AllianceFlipUtil.apply(new Translation2d(7.18, 3.317)))));

    NamedCommands.registerCommand("ResetOdometry Center XY With Vision",
        new AutoUpdateOdometry(drivetrain, new Translation2d(7.18, 4)));

    NamedCommands.registerCommand("ResetOdometry Right XY With Vision",
        new AutoUpdateOdometry(drivetrain, new Translation2d(7.18, 3.317)));
    NamedCommands.registerCommand("ResetOdometry Left XY With Vision",
        new AutoUpdateOdometry(drivetrain, new Translation2d(7.18, 4.73)));
    NamedCommands.registerCommand("Intake Coral Optimized", new CollectAutonomousOpitimized(superStructure));
    NamedCommands.registerCommand("Intake Coral",
        new CollectAutonomous(superStructure).andThen(new DefaultPosition(superStructure))
            .finallyDo(() -> superStructure.scorer.stopEndEffector()));
    NamedCommands.registerCommand("Score Coral A",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.A));
    NamedCommands.registerCommand("Score Coral B",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.B));
    NamedCommands.registerCommand("Score Coral C",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.C));
    NamedCommands.registerCommand("Score Coral D",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.D));
    NamedCommands.registerCommand("Score Coral E",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.E));
    NamedCommands.registerCommand("Score Coral F",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.F));
    NamedCommands.registerCommand("Score Coral G",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.G));
    NamedCommands.registerCommand("Score Coral H",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.H));
    NamedCommands.registerCommand("Score Coral I",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.I));
    NamedCommands.registerCommand("Score Coral J",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.J));
    NamedCommands.registerCommand("Score Coral K",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.K));
    NamedCommands.registerCommand("Score Coral L",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.L));

    NamedCommands.registerCommand("Score Coral A Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.A));
    NamedCommands.registerCommand("Score Coral B Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.B));
    NamedCommands.registerCommand("Score Coral C Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.C));
    NamedCommands.registerCommand("Score Coral D Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.D));
    NamedCommands.registerCommand("Score Coral E Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.E));
    NamedCommands.registerCommand("Score Coral F Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.F));
    NamedCommands.registerCommand("Score Coral G Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.G));
    NamedCommands.registerCommand("Score Coral H Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.H));
    NamedCommands.registerCommand("Score Coral I Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.I));
    NamedCommands.registerCommand("Score Coral J Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.J));
    NamedCommands.registerCommand("Score Coral K Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.K));
    NamedCommands.registerCommand("Score Coral L Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.L));

    NamedCommands.registerCommand("Score Coral A Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.A));
    NamedCommands.registerCommand("Score Coral B Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.B));
    NamedCommands.registerCommand("Score Coral C Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.C));
    NamedCommands.registerCommand("Score Coral D Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.D));
    NamedCommands.registerCommand("Score Coral E Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.E));
    NamedCommands.registerCommand("Score Coral F Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.F));
    NamedCommands.registerCommand("Score Coral G Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.G));
    NamedCommands.registerCommand("Score Coral H Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.H));
    NamedCommands.registerCommand("Score Coral I Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.I));
    NamedCommands.registerCommand("Score Coral J Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.J));
    NamedCommands.registerCommand("Score Coral K Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.K));
    NamedCommands.registerCommand("Score Coral L Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.L));

    NamedCommands.registerCommand("Score Coral A Optimized Direct Without Backup",
        new AutoScoreCoralAutonomousOptimizedDirectWithoutBackup(superStructure, drivetrain, TargetBranch.A));
    NamedCommands.registerCommand("Score Coral B Optimized Direct Without Backup",
        new AutoScoreCoralAutonomousOptimizedDirectWithoutBackup(superStructure, drivetrain, TargetBranch.B));
    NamedCommands.registerCommand("Score Coral C Optimized Direct Without Backup",
        new AutoScoreCoralAutonomousOptimizedDirectWithoutBackup(superStructure, drivetrain, TargetBranch.C));
    NamedCommands.registerCommand("Score Coral D Optimized Direct Without Backup",
        new AutoScoreCoralAutonomousOptimizedDirectWithoutBackup(superStructure, drivetrain, TargetBranch.D));
    NamedCommands.registerCommand("Score Coral E Optimized Direct Without Backup",
        new AutoScoreCoralAutonomousOptimizedDirectWithoutBackup(superStructure, drivetrain, TargetBranch.E));
    NamedCommands.registerCommand("Score Coral F Optimized Direct Without Backup",
        new AutoScoreCoralAutonomousOptimizedDirectWithoutBackup(superStructure, drivetrain, TargetBranch.F));
    NamedCommands.registerCommand("Score Coral G Optimized Direct Without Backup",
        new AutoScoreCoralAutonomousOptimizedDirectWithoutBackup(superStructure, drivetrain, TargetBranch.G));
    NamedCommands.registerCommand("Score Coral H Optimized Direct Without Backup",
        new AutoScoreCoralAutonomousOptimizedDirectWithoutBackup(superStructure, drivetrain, TargetBranch.H));
    NamedCommands.registerCommand("Score Coral I Optimized Direct Without Backup",
        new AutoScoreCoralAutonomousOptimizedDirectWithoutBackup(superStructure, drivetrain, TargetBranch.I));
    NamedCommands.registerCommand("Score Coral J Optimized Direct Without Backup",
        new AutoScoreCoralAutonomousOptimizedDirectWithoutBackup(superStructure, drivetrain, TargetBranch.J));
    NamedCommands.registerCommand("Score Coral K Optimized Direct Without Backup",
        new AutoScoreCoralAutonomousOptimizedDirectWithoutBackup(superStructure, drivetrain, TargetBranch.K));
    NamedCommands.registerCommand("Score Coral L Optimized Direct Without Backup",
        new AutoScoreCoralAutonomousOptimizedDirectWithoutBackup(superStructure, drivetrain, TargetBranch.L));

    NamedCommands.registerCommand("Move To Default",
        new DefaultPositionAutonomous(superStructure));

    NamedCommands.registerCommand("Set Coral Level L1",
        new InstantCommand(() -> this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L1)));
    NamedCommands.registerCommand("Set Coral Level L2",
        new InstantCommand(() -> this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L2)));
    NamedCommands.registerCommand("Set Coral Level L3",
        new InstantCommand(() -> this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L3)));
    NamedCommands.registerCommand("Set Coral Level L4",
        new InstantCommand(() -> this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L4)));
  }

  private void configureBindings() {

    keyBoard.collectCoral().onTrue(
        new CollectPosition(superStructure, drivetrain).deadlineFor(new SwerveAngleWithCoralStation(drivetrain)));

    keyBoard.prepareToScore().onTrue(new ScoreObjectPosition(superStructure, drivetrain));

    keyBoard.scoreObject().and(() -> superStructure.scorer.readyToScoreProcessor())
        .onTrue(new ScoreProcessor(superStructure));

    keyBoard.reefL1()
        .onTrue(new InstantCommand(() -> {
          this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L1);
          this.superStructure.scorer.setTargetAlgaeHeight(AlgaeHeightReef.GROUND);
          this.superStructure.scorer.setAlgaeManualControl(true);
        }));

    keyBoard.reefL2()
        .onTrue(new InstantCommand(() -> {
          this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L2);
          this.superStructure.scorer.setTargetAlgaeHeight(AlgaeHeightReef.LOW);
          this.superStructure.scorer.setAlgaeManualControl(true);
        }));

    keyBoard.reefL3()
        .onTrue(new InstantCommand(() -> {
          this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L3);
          this.superStructure.scorer.setTargetAlgaeHeight(AlgaeHeightReef.MID);
          this.superStructure.scorer.setAlgaeManualControl(true);
        }));

    keyBoard.reefL4()
        .onTrue(new InstantCommand(() -> this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L4)));

    keyBoard.removeAlgaeFromBranch().and(() -> this.superStructure.scorer.isAlgaeManualControl())
        .onTrue(new RemoveAlgaePosition(superStructure, drivetrain)
            .deadlineFor(Commands.run(() -> drivetrain.driveAlignAngleJoystickRemoveAlgae(), drivetrain)));

    keyBoard.alignToClimb().onTrue(new ClimbPosition(superStructure)
        .deadlineFor(Commands.runOnce(() -> drivetrain.setAngleForClimb())
            .andThen(Commands.run(() -> drivetrain.driveLockedAngleToClimb(), drivetrain)
                .until(() -> superStructure.robotIsClimbed)
                .andThen(Commands.run(() -> drivetrain.stopSwerve(), drivetrain)))));

    keyBoard.cancelAction().onTrue(new DefaultPosition(superStructure));

    keyBoard.goToReefA().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.A)
        .alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.A);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefB().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.B)
        .alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.B);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefC().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.C).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.C);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefD().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.D).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.D);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefE().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.E).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.E);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefF().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.F).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.F);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefG().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.G).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.G);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefH().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.H).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.H);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefI().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.I).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.I);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefJ().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.J).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.J);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefK().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.K).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.K);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    keyBoard.goToReefL().onTrue(
        new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.L).alongWith(new InstantCommand(() -> {
          superStructure.scorer.setAutoAlgaeLevel(TargetBranch.L);
          superStructure.scorer.setAlgaeManualControl(false);
        })));

    driverController.x().and(() -> DriverStation.isDisabled()).whileTrue(Commands
        .runEnd(() -> superStructure.setCoastToRobot(), () -> superStructure.setBrakeToRobot()).ignoringDisable(true));
    drivetrain.registerTelemetry(logger::telemeterize);

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
