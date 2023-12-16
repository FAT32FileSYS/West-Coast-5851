// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;

import frc.robot.Constants;
import frc.robot.subsystems.drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveFoward extends SequentialCommandGroup {
  /** Creates a new MoveFoward. */
  public MoveFoward() {
    public MoveFoward(drive s_drive)
    {
      ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Straight",
      new PathConstraints(6, 2)
      );
      
      // This is just an example event map. It would be better to have a constant, global event map
      // in your code that will be used by all path following commands.
      HashMap<String, Command> eventMap = new HashMap<>();
      // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
      BaseAutoBuilder autoBuilder = new BaseAutoBuilder(
          s_drive::getPose, // Pose2d supplier
          s_drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
          s_drive::
          new PIDConstants(Constants.AutoConstants.kPXController, 0.0, Constants.AutoConstants.kDXController), // PID constants to correct for translation error (used to create the X and Y PID controllers)
          s_drive::setModuleStates, // Module states consumer used to output to the drive subsystem
          eventMap,
          true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true // The drive subsystem. Used to properly set the requirements of path following commands
      );
      
          autoBuilder.fullAuto(pathGroup).schedule();
  }
}

