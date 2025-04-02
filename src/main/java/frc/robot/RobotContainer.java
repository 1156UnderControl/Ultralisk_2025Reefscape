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
import frc.Java_Is_UnderControl.LEDs.ILed;
import frc.Java_Is_UnderControl.LEDs.LedSubsystem;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.robot.commands.autonomous_commands.AutoScoreCoralAutonomous;
import frc.robot.commands.autonomous_commands.AutoScoreCoralAutonomousOptimized;
import frc.robot.commands.autonomous_commands.AutoScoreCoralAutonomousOptimizedDirect;
import frc.robot.commands.autonomous_commands.AutoUpdateOdometry;
import frc.robot.commands.autonomous_commands.CollectAutonomous;
import frc.robot.commands.autonomous_commands.CollectAutonomousOpitimized;
import frc.robot.commands.autonomous_commands.DefaultPositionAutonomous;
import frc.robot.commands.states.AutoScoreCoralPosition;
import frc.robot.commands.states.ClimbPosition;
import frc.robot.commands.states.CollectPosition;
import frc.robot.commands.states.DefaultPosition;
import frc.robot.commands.states.RemoveAlgaePosition;
import frc.robot.commands.states.ScoreAlgaeNetPosition;
import frc.robot.commands.states.ScoreCoralPosition;
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

  private SwerveModuleConstants[] modulosArray = TunerConstants.getModuleConstants();

  public final SwerveSubsystem drivetrain = new SwerveSubsystem(() -> superStructure.scorer.isElevatorInHighPosition(),
      () -> superStructure.scorer.getTargetReefLevel(),
      TunerConstants.getSwerveDrivetrainConstants(),
      modulosArray[0], modulosArray[1], modulosArray[2], modulosArray[3]);

  private final Telemetry logger = new Telemetry(drivetrain.MaxSpeed);

  public final ILed leds = LedSubsystem.getInstance();

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
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.A)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral B",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.B)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral C",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.C)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral D",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.D)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral E",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.E)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral F",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.F)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral G",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.G)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral H",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.H)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral I",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.I)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral J",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.J)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral K",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.K)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral L",
        new AutoScoreCoralAutonomous(superStructure, drivetrain, TargetBranch.L)
            .andThen(new DefaultPositionAutonomous(superStructure)));

    NamedCommands.registerCommand("Score Coral A Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.A)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral B Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.B)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral C Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.C)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral D Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.D)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral E Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.E)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral F Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.F)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral G Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.G)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral H Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.H)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral I Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.I)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral J Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.J)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral K Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.K)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral L Optimized",
        new AutoScoreCoralAutonomousOptimized(superStructure, drivetrain, TargetBranch.L)
            .andThen(new DefaultPositionAutonomous(superStructure)));

    NamedCommands.registerCommand("Score Coral A Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.A)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral B Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.B)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral C Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.C)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral D Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.D)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral E Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.E)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral F Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.F)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral G Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.G)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral H Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.H)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral I Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.I)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral J Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.J)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral K Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.K)
            .andThen(new DefaultPositionAutonomous(superStructure)));
    NamedCommands.registerCommand("Score Coral L Optimized Direct",
        new AutoScoreCoralAutonomousOptimizedDirect(superStructure, drivetrain, TargetBranch.L)
            .andThen(new DefaultPositionAutonomous(superStructure)));

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

    keyBoard.collectCoral().onTrue(new CollectPosition(superStructure, drivetrain));

    keyBoard.prepareToScoreCoral().onTrue(new ScoreCoralPosition(superStructure, drivetrain));

    keyBoard.reefL1()
        .onTrue(new InstantCommand(() -> {
          this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L1);
          this.superStructure.scorer.setTargetAlgaeHeight(AlgaeHeightReef.LOW);
        }));

    keyBoard.reefL2()
        .onTrue(new InstantCommand(() -> {
          this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L2);
          this.superStructure.scorer.setTargetAlgaeHeight(AlgaeHeightReef.MID);
        }));

    keyBoard.reefL3()
        .onTrue(new InstantCommand(() -> this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L3)));

    keyBoard.reefL4()
        .onTrue(new InstantCommand(() -> this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L4)));

    keyBoard.removeAlgaeFromBranch()
        .onTrue(new RemoveAlgaePosition(superStructure, drivetrain)
            .deadlineFor(Commands.run(() -> drivetrain.driveAlignAngleJoystickRemoveAlgae(), drivetrain)));

    keyBoard.scoreAlgae()
        .onTrue(new ScoreAlgaeNetPosition(superStructure, drivetrain));

    keyBoard.alignToClimb().onTrue(new ClimbPosition(superStructure)
        .deadlineFor(Commands.runOnce(() -> drivetrain.setAngleForClimb())
            .andThen(Commands.run(() -> drivetrain.driveLockedAngleToClimb(), drivetrain)
                .until(() -> superStructure.robotIsClimbed)
                .andThen(Commands.run(() -> drivetrain.stopSwerve(), drivetrain)))));

    keyBoard.cancelAction().onTrue(new DefaultPosition(superStructure));

    keyBoard.goToReefA().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.A));

    keyBoard.goToReefB().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.B));

    keyBoard.goToReefC().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.C));

    keyBoard.goToReefD().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.D));

    keyBoard.goToReefE().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.E));

    keyBoard.goToReefF().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.F));

    keyBoard.goToReefG().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.G));

    keyBoard.goToReefH().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.H));

    keyBoard.goToReefI().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.I));

    keyBoard.goToReefJ().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.J));

    keyBoard.goToReefK().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.K));

    keyBoard.goToReefL().onTrue(new AutoScoreCoralPosition(superStructure, drivetrain, TargetBranch.L));

    driverController.x().and(() -> DriverStation.isDisabled())
        .whileTrue(Commands.runEnd(() -> superStructure.setCoastToRobot(), () -> superStructure.setBrakeToRobot())
            .ignoringDisable(true));
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
