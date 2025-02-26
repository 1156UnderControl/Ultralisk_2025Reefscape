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
import frc.robot.joysticks.DriverController;
import frc.robot.joysticks.OperatorController;
import frc.robot.states.CollectState;
import frc.robot.states.DefaultState;
import frc.robot.states.RemoveAlgaeState;
import frc.robot.states.ScoreCoralState;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private OperatorController keyBoard = OperatorController.getInstance();

  private DriverController driverController = DriverController.getInstance();

  private SwerveModuleConstants[] modulosArray = TunerConstants.getModuleConstants();

  public final SwerveSubsystem drivetrain = new SwerveSubsystem(TunerConstants.getSwerveDrivetrainConstants(),
      modulosArray[0], modulosArray[1], modulosArray[2], modulosArray[3]);

  private final Telemetry logger = new Telemetry(drivetrain.MaxSpeed);

  public final SuperStructure superStructure = new SuperStructure();

  public RobotContainer() {
    configureBindings();
    this.autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);
    superStructure.setDefaultCommand(new DefaultState(superStructure));
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(
        Commands.run(() -> drivetrain.driveAlignAngleJoy(), drivetrain).onlyIf(() -> DriverStation.isTeleopEnabled()));

    driverController.rightBumper().whileTrue(drivetrain.wheelRadiusCharacterization());

    keyBoard.collectCoral().onTrue(new CollectState(superStructure, drivetrain));

    keyBoard.prepareToScoreCoral().onTrue(new ScoreCoralState(superStructure, drivetrain));

    keyBoard.removeAlgaeFromBranch()
        .onTrue(new RemoveAlgaeState(superStructure, drivetrain));

    keyBoard.cancelAction().onTrue(new DefaultState(superStructure));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
