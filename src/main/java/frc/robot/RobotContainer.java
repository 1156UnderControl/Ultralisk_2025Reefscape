// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.Java_Is_UnderControl.LEDs.ILed;
import frc.Java_Is_UnderControl.LEDs.LedSubsystem;
import frc.robot.commands.states.AutoScoreCoralPosition;
import frc.robot.commands.states.ClimbPosition;
import frc.robot.commands.states.CollectPosition;
import frc.robot.commands.states.DefaultPosition;
import frc.robot.commands.states.RemoveAlgaePosition;
import frc.robot.commands.states.ScoreCoralPosition;
import frc.robot.constants.FieldConstants.AlgaeHeight;
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
    configureBindings();
    this.autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);
    superStructure.setDefaultCommand(new DefaultPosition(superStructure));
    drivetrain.setDefaultCommand(
        Commands.run(() -> drivetrain.driveAlignAngleJoystick(), drivetrain)
            .onlyIf(() -> DriverStation.isTeleopEnabled()));
  }

  private void configureBindings() {

    // driverController.rightBumper().onTrue(new
    // AutoIntakeCoralPosition(superStructure, drivetrain));

    keyBoard.collectCoral().onTrue(new CollectPosition(superStructure, drivetrain));

    keyBoard.prepareToScoreCoral().onTrue(new ScoreCoralPosition(superStructure, drivetrain));

    driverController.x().whileTrue(
        Commands.runEnd(() -> superStructure.scorer.setCoastScorer(), () -> superStructure.scorer.setBrakeScorer())
            .ignoringDisable(true));

    keyBoard.reefL1()
        .onTrue(new InstantCommand(() -> {
          this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L1);
          this.superStructure.scorer.setTargetAlgaeHeight(AlgaeHeight.LOW);
        }));

    keyBoard.reefL2()
        .onTrue(new InstantCommand(() -> {
          this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L2);
          this.superStructure.scorer.setTargetAlgaeHeight(AlgaeHeight.MID);
        }));

    keyBoard.reefL3()
        .onTrue(new InstantCommand(() -> this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L3)));

    keyBoard.reefL4()
        .onTrue(new InstantCommand(() -> this.superStructure.scorer.setTargetBranchLevel(ReefLevel.L4)));

    keyBoard.removeAlgaeFromBranch()
        .onTrue(new RemoveAlgaePosition(superStructure, drivetrain));

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
