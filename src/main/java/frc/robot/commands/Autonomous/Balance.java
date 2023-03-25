// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Balance extends CommandBase {
  /** Creates a new Balance. */
  Drivetrain m_drive;
  double yAxisRate;
  double xAxisRate;

  public Balance() {
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchAngleDegrees = m_drive.getPitch();
    double rollAngleDegrees = m_drive.getRoll();

    // Control drive system automatically, 
    // driving in reverse direction of pitch/roll angle,
    // with a magnitude based upon the angle
    
    if ( Math.abs(pitchAngleDegrees) > Math.abs(Constants.BalanceConstants.kOffBalanceAngleThresholdDegrees) ) {
        double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
        xAxisRate = Math.sin(pitchAngleRadians) * -1;
    }
    else if ( Math.abs(pitchAngleDegrees) < Math.abs(Constants.BalanceConstants.kOffBalanceAngleThresholdDegrees) ){
      double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
      xAxisRate = Math.sin(pitchAngleRadians);
    }

    if ( (Math.abs(pitchAngleDegrees) > Math.abs(Constants.BalanceConstants.kOffBalanceAngleThresholdDegrees)) ) {
        double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
        yAxisRate = Math.sin(rollAngleRadians) * -1;
    }
    else if ((Math.abs(pitchAngleDegrees) < Math.abs(Constants.BalanceConstants.kOffBalanceAngleThresholdDegrees))) {
      double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
      yAxisRate = Math.sin(rollAngleRadians);
    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.getPitch() == Constants.BalanceConstants.kOffBalanceAngleThresholdDegrees && m_drive.getRoll() == Constants.BalanceConstants.kOffBalanceAngleThresholdDegrees;
  }
}
