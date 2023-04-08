// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class PrintingCommand extends CommandBase {
    private Drivetrain m_drive;

    public PrintingCommand(){
        //System.out.println("Scheduler Works");
       m_drive.arcadeDrive(0.5, 0);
    }
}
