// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Autonomous;

// import com.pathplanner.lib.PathPlannerTrajectory;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.RamseteController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.CascadeConstants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Cascade;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Intake;

// public class Scoring extends CommandBase {
//   /** Creates a new Scoring. */
//   private final Drivetrain m_drive = new Drivetrain();
//   private final Cascade m_cascade = new Cascade();
//   private final Arm m_arm = new Arm();
//   private final Intake m_intake = new Intake();

//   public Scoring() {
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }

//   public Command SimpleScoringSequence(){
//     m_cascade.autoCascadeDrive(CascadeConstants.kstage3);
//   }

//   public Command baseDriveCommand(PathPlannerTrajectory traj){
    
//     var autoVoltageConstraint =
//     new DifferentialDriveVoltageConstraint(
//         new SimpleMotorFeedforward(
//             DriveConstants.ksVolts,
//             DriveConstants.kvVoltSecondsPerMeter,
//             DriveConstants.kaVoltSecondsSquaredPerMeter),
//         DriveConstants.kDriveKinematics,
//         12.5);

// // Create config for trajectory
// TrajectoryConfig config =
//     new TrajectoryConfig(
//             AutoConstants.kMaxSpeedMetersPerSecond,
//             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//         // Add kinematics to ensure max speed is actually obeyed
//         .setKinematics(DriveConstants.kDriveKinematics)
//         // Apply the voltage constraint
//         .addConstraint(autoVoltageConstraint);

// // An example trajectory to follow.  All units in meters.
// RamseteCommand ramseteCommand =
//     new RamseteCommand(
//         traj,
//         m_drive::getPose,
//         new RamseteController(
//             AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
//         new SimpleMotorFeedforward(
//             DriveConstants.ksVolts,
//             DriveConstants.kvVoltSecondsPerMeter,
//             DriveConstants.kaVoltSecondsSquaredPerMeter),
//         DriveConstants.kDriveKinematics,
//         m_drive::getWheelSpeeds,
//         new PIDController(DriveConstants.kPDriveVel, 0, 0),
//         new PIDController(DriveConstants.kPDriveVel, 0, 0),
//         // RamseteCommand passes volts to the callback
//         m_drive::tankDriveVolts,
//         m_drive);

// // Reset odometry to starting pose of trajectory.
// m_drive.resetOdometry(traj.getInitialPose());

// return Commands.sequence(ramseteCommand);

// // Run path following command, then stop at the end.

//   }


// }
