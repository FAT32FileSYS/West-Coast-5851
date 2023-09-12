// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.commands.FlyWheelCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Intake;
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
  private final Intake intakeSub = new Intake();
  private final FlyWheel flyWheelSub = new FlyWheel();

  private final Joystick armando = new Joystick(DriverConstants.armando);

  private final IntakeCommand intakeCommand = new IntakeCommand(DriverConstants.intakeSpeed, intakeSub);
  private final IntakeCommand intakeCommandNeg = new IntakeCommand(-DriverConstants.intakeSpeed, intakeSub);

  private final FlyWheelCommand flyWheelCommand = new FlyWheelCommand(DriverConstants.flyWheel, flyWheelSub);

  private RunCommand armandoMove = new RunCommand(() -> driveSub.move(DriverConstants.driveSpeed * armando.getRawAxis(DriverConstants.leftAxis),
     DriverConstants.driveSpeed * armando.getRawAxis(DriverConstants.rightAxis)), driveSub);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    driveSub.setDefaultCommand(armandoMove);

    JoystickButton intakeRun = new JoystickButton(armando, DriverConstants.intakeStart);
    intakeRun.whileTrue(intakeCommand);

    JoystickButton flywheelRun = new JoystickButton(armando, DriverConstants.flywheelStart);
    flywheelRun.whileTrue(flyWheelCommand);

    JoystickButton intakeRev = new JoystickButton(armando, DriverConstants.intakeRev);
    intakeRev.whileTrue(intakeCommandNeg);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
