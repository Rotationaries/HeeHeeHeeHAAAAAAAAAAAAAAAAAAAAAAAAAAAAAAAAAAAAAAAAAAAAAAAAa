// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;


/** An example command that uses an example subsystem. */
public class FollowingPath extends CommandBase {
  //private Trajectory traj;
  private Drivetrain m_drive;
  //private int pcx, pcy;
  private String fileName;
  private PathPlannerTrajectory traj;

  //private double ks, kv, ka;

  public FollowingPath(PathPlannerTrajectory traj) {
    this.traj = traj;

    //traj = PathPlanner.generatePath(new PathConstraints(3.0, 4.0), pointlist);

    //An example trajectory to follow.  All units in meters
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    followPath();
  }

  public void followPath() {
    //feedforward.calculate(pc.getVelo(), pc.getVelo()+pc.getPID().getVelocityError(), pc.getPID().getPeriod());

    //Supplier<Pose2d> supply = () -> track.getPos();
    CommandScheduler.getInstance().schedule(new PPRamseteCommand(traj, m_drive::getPose, AutoConstants.controller, 
    AutoConstants.feedforward, DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds, 
    new PIDController(DriveConstants.kPDriveVel, 0, 0), 
    new PIDController(DriveConstants.kPDriveVel, 0, 0), m_drive::tankDriveVolts, m_drive));
  }

  public Trajectory getTraj() {
    return traj;
  }

  public String fileName() {
     return fileName;
  }

  /*public List<PathPlannerTrajectory> getPaths() {
    return paths;
  }*/

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /*public CommandBase chargingStartion(){
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("DirectDock-M", 0,0);
    PathPlannerTrajectory traj = PathPlanner.loadPath("TestNew", AutoConstants.kMaxSpeedMetersPerSecond, 
      AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    RamseteCommand command = new RamseteCommand(traj, m_robotDrive::getPose, new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
    AutoConstants.feedforward, DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds, 
    x, y, m_robotDrive::tankDriveVolts, m_robotDrive); 
    return Commands.sequence()
  }*/

}
