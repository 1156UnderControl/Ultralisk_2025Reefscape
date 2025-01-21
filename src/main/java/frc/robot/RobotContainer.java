// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.Java_Is_UnderControl.Joysticks.Types.TypeXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {

  TypeXboxController controllerType;
  ArmSubsystem armSubsystem;

  public RobotContainer() {
    armSubsystem = ArmSubsystem.getInstance();
    controllerType = new TypeXboxController(0);
    configureBindings();
  }

  private void configureBindings() {
    controllerType.buttomUp().onTrue(this.armSubsystem.sysIdQuasistaticForward());
    controllerType.buttomDown().onTrue(this.armSubsystem.sysIdQuasistaticReverse());
    controllerType.buttomRight().onTrue(this.armSubsystem.sysIdDynamicForward());
    controllerType.buttomLeft().onTrue(this.armSubsystem.sysIdDynamicReverse());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
