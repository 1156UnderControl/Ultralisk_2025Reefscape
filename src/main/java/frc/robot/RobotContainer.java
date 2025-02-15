// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.Java_Is_UnderControl.Util.AllianceFlipUtil;
import frc.Java_Is_UnderControl.Util.CoordinatesTransform;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.ReefHeight;
import frc.robot.joysticks.ControlBoard;
import frc.robot.joysticks.OperatorController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private ControlBoard driverController = ControlBoard.getInstance();
  private OperatorController operatorPanel = OperatorController.getInstance();

  private SwerveModuleConstants[] modulosArray = TunerConstants.getModuleConstants();

  public final SwerveSubsystem drivetrain = new SwerveSubsystem(TunerConstants.getSwerveDrivetrainConstants(),
      modulosArray[0], modulosArray[1], modulosArray[2], modulosArray[3]);

  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(drivetrain.MaxSpeed);

  public final SuperStructure superStructure = new SuperStructure();

  public RobotContainer() {
    configureBindings();
    this.autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);
  }

  private void configureBindings() {
    Pose3d posebranch1Score = CoordinatesTransform
        .getRetreatPose(AllianceFlipUtil.apply(Reef.branchPositions.get(1).get(ReefHeight.L2)), 1.0);
    Pose3d posebranch7Score = CoordinatesTransform
        .getRetreatPose(AllianceFlipUtil.apply(Reef.branchPositions.get(7).get(ReefHeight.L2)), 1.0);

    // drivetrain.setDefaultCommand(
    // Commands.run(() -> drivetrain.driveAlignAngleJoy(), drivetrain).onlyIf(() ->
    // DriverStation.isTeleopEnabled()));

    driverController.a()
        .whileTrue(Commands.runEnd(() -> superStructure.scorer.setElevatorDutyCycle(1),
            () -> superStructure.scorer.setElevatorDutyCycle(0.0), superStructure));

    driverController.b()
        .whileTrue(Commands.runEnd(() -> superStructure.scorer.setElevatorDutyCycle(-0.5),
            () -> superStructure.scorer.setElevatorDutyCycle(0.0), superStructure));

    // driverController.b().whileTrue(drivetrain.wheelRadiusCharacterization());

    // operatorPanel.goToReefB()
    // .onTrue(
    // drivetrain.goToPoseWithPathfind(posebranch1Score));
    // operatorPanel.goToReefG()
    // .onTrue(
    // drivetrain.goToPoseWithPathfind(posebranch7Score));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
