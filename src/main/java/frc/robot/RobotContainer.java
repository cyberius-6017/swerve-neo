// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.newDriveTrainCommand;
import frc.robot.subsystems.Drivetrain.newDriveTrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  private final XboxController driverController = new XboxController(Constants.OperatorConstants.driverDriveTrainPort);
  private final newDriveTrain m_NewDriveTrain = new newDriveTrain();

  public RobotContainer() {
    
    m_NewDriveTrain.setDefaultCommand(new newDriveTrainCommand(m_NewDriveTrain, 
                                                               ()-> -driverController.getLeftX(), 
                                                               ()-> driverController.getLeftY(),
                                                               ()-> -driverController.getRightX(),
                                                               ()-> driverController.getBButtonPressed(),
                                                               ()-> driverController.getYButtonPressed(),
                                                               ()-> driverController.getRightTriggerAxis(),
                                                               ()-> driverController.getLeftTriggerAxis()));
  }
}
