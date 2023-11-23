// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.CascadeCommand;
import frc.robot.commands.ClawArmCommand;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.autoDrivePIDCommand;
import frc.robot.subsystems.Cascade;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClawArm;
import frc.robot.subsystems.drive;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final drive driveSub = new drive();
  private final Claw claw = new Claw();
  private final ClawArm clawArm = new ClawArm();
  private final Cascade cascade = new Cascade();


  private final Joystick armando = new Joystick(DriverConstants.armando);

 

  private RunCommand armandoMove = new RunCommand(() -> driveSub.move(DriverConstants.driveSpeed * armando.getRawAxis(DriverConstants.leftAxis),
     DriverConstants.driveSpeed * armando.getRawAxis(DriverConstants.rightAxis)), driveSub);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    driveSub.setDefaultCommand(armandoMove);



  }

  
  private void configureBindings() {
    
  }
  
  public Command getAutonomousCommand() {
    return null;
  }
}
