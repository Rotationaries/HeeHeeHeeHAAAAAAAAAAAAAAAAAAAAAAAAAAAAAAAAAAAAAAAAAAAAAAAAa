/*// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Turn extends CommandBase {
  /** Creates a new SkyBalance. */
  /*Drivetrain m_drive;
  PIDController controller = new PIDController(0.05, 0, 0);
  private double startTime;
  private double time;

  public Turn(Drivetrain m_drive, double heading) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.time = time;
    this.startTime = Timer.getFPGATimestamp();
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double turn = controller.calculate(m_drive.getHeading());
    //turn = -MathUtil.clamp(turn,-1,1);
    m_drive.arcadeDrive(0, 0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.getHeading() == 180;
  }
}*/

