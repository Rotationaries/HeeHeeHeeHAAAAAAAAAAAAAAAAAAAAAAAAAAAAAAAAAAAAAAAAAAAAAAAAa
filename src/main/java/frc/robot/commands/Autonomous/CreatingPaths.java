// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

/** An example command that uses an example subsystem. */
public class CreatingPaths extends CommandBase {
  private Drivetrain m_drive;
  private Trajectory traj;
  //private int pcx, pcy;
  private String fileName;

  //private double ks, kv, ka;

  public CreatingPaths(Drivetrain drive, String fileName) {
    m_drive = drive;
    this.fileName = fileName;

    //traj = PathPlanner.generatePath(new PathConstraints(3.0, 4.0), pointlist);

    //An example trajectory to follow.  All units in meters
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    traj = PathPlanner.loadPath(fileName, new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, 
    AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    followPath();
  }

  public Command followPath() {
    //feedforward.calculate(pc.getVelo(), pc.getVelo()+pc.getPID().getVelocityError(), pc.getPID().getPeriod());

    //Supplier<Pose2d> supply = () -> track.getPos();

    return new RamseteCommand(traj, m_drive::getPose, AutoConstants.controller, 
    AutoConstants.feedForward, DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds, 
    new PIDController(DriveConstants.kPDriveVel, 0, 0), 
    new PIDController(DriveConstants.kPDriveVel, 0, 0), m_drive::tankDriveVolts, m_drive);
  }

  public Trajectory getTraj() {
    return traj;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

