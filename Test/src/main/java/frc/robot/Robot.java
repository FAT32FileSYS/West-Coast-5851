  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.autoDrivePIDCommand;
import frc.robot.subsystems.drive;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.commands.autoDrivePIDCommand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private autoDrivePIDCommand autodrivePID;
  double pastTime;
  
  private RobotContainer m_robotContainer;
  private drive drive = new drive();

  String trajectoryJSON = "src/main/deploy/pathplanner/generatedJSON/pathfinder.wpilib.json";
  Trajectory trajectory = new Trajectory();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    autodrivePID = new autoDrivePIDCommand(); //for variables


    

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    pastTime = Timer.getFPGATimestamp();
    drive.leftSideEncoder = 0;
    drive.rightSideEncoder = 0;
    autodrivePID.leftPastError = 0;
    autodrivePID.rightPastError = 0;
    autodrivePID.leftMotorOutput = 0;
    autodrivePID.rightMotorOutput = 0;
    autodrivePID.leftPastError = 0;
    autodrivePID.rightPastError = 0;
    autodrivePID.rightCurrentLocation = 0;
    autodrivePID.leftCurrentLocation = 0;
    autodrivePID.leftTotalIntegral = 0;
    autodrivePID.rightTotalIntegral = 0;

    if (autodrivePID != null)
    {
      autodrivePID.schedule();
    }
  
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() 
  {
    //drive.drive.tankDrive(autodrivePID.leftMotorOutput, autodrivePID.rightMotorOutput);
    drive.rightFront.set(-autodrivePID.rightMotorOutput);
    drive.rightBack.set(-autodrivePID.rightMotorOutput);
    drive.leftBack.set(autodrivePID.leftMotorOutput);
    drive.leftBack.set(autodrivePID.leftMotorOutput);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
