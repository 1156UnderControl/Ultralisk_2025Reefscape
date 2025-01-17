// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.Java_Is_UnderControl.Joysticks.Types.TypeXboxController;
import frc.robot.commands.DynamicForward;
import frc.robot.commands.DynamicReverse;
import frc.robot.commands.QuasistaticForward;
import frc.robot.commands.QuasistaticReverse;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {

  TypeXboxController controllerType;
  ArmSubsystem armSubsystem;

  public RobotContainer() {
    configureBindings();
    controllerType = new TypeXboxController(0);
    armSubsystem = ArmSubsystem.getInstance();
  }

  private void configureBindings() {
    controllerType.buttomUp().onTrue(new QuasistaticForward(armSubsystem));
    controllerType.buttomDown().onTrue(new QuasistaticReverse(armSubsystem));
    controllerType.buttomRight().onTrue(new DynamicForward(armSubsystem));
    controllerType.buttomLeft().onTrue(new DynamicReverse(armSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
