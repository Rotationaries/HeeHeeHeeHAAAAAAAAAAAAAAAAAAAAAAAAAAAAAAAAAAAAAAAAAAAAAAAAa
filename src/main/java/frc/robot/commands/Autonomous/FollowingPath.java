// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;


/** An example command that uses an example subsystem. */
public class FollowingPath extends CommandBase {
  //private Trajectory traj;
  private Drivetrain m_drive;
  //private int pcx, pcy;
  private String fileName;
  private PathPlannerTrajectory traj;

  //private double ks, kv, ka;

  public FollowingPath(Drivetrain m_drive){
    this.m_drive = m_drive;
    

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
    
  }

  public Command autoPath() {
    //feedforward.calculate(pc.getVelo(), pc.getVelo()+pc.getPID().getVelocityError(), pc.getPID().getPeriod());

    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        12.5);

// Create config for trajectory
TrajectoryConfig config =
    new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

// An example trajectory to follow.  All units in meters.

PathPlannerTrajectory traj = PathPlanner.loadPath("DirectDock-M Blue", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));


// Trajectory exampleTrajectory =
//     TrajectoryGenerator.generateTrajectory(
//         // Start at (1, 2) facing the +X direction
//         new Pose2d(1, 2, new Rotation2d(0)),
//         // Pass through these two interior waypoints, making an 's' curve path
//         List.of(new Translation2d(2, 3), new Translation2d(3, 1)),
//         // End 3 meters straight ahead of where we started, facing forward
//         new Pose2d(4, 2, new Rotation2d(0)),
//         // Pass config
//         config);

RamseteCommand ramseteCommand =
    new RamseteCommand(
        traj,
        m_drive::getPose,
        new RamseteController(
            AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_drive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drive::tankDriveVolts,
        m_drive);

// Reset odometry to starting pose of trajectory.
m_drive.resetOdometry(traj.getInitialPose());

// Run path following command, then stop at the end.
return ramseteCommand.andThen(() -> new SkyBalance(m_drive).andThen(() -> m_drive.tankDriveVolts(0, 0)));
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
