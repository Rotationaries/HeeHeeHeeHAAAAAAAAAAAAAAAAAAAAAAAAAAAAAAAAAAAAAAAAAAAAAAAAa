// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private final NetworkTable m_limelightTable;
  private final double m_isTargetValid;
  double[] targetPose = new double[6];


  // private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // private NetworkTableEntry fieldPosition = table.getEntry("botpose_wpired");
  // private NetworkTableEntry tx = table.getEntry("tx");
  // private NetworkTableEntry ty = table.getEntry("ty");
  // private NetworkTableEntry tv = table.getEntry("tv");
  // private NetworkTableEntry pipeline = table.getEntry("pipeline");
  // private NetworkTableEntry stream = table.getEntry("stream");



  public Vision() {
    //SmartDashboard.putNumber()
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    m_isTargetValid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    
    
  }

  public boolean getValidTarget(){
    if (m_isTargetValid == 1){
      return(true);
    }
    else{
      return(false);
    }
  }

  public double getTX(){
    targetPose = m_limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (targetPose[0]);
  }

  public double getTY(){
    targetPose = m_limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (targetPose[1]);
  }

  public double getTZ(){
    targetPose = m_limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (targetPose[2]);
  }

  public double getRX(){
    targetPose = m_limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (targetPose[3]);
  }

  public double getRY(){
    targetPose = m_limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (targetPose[4]);
  }
  
  public double getRZ(){
    targetPose = m_limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (targetPose[5]);
  }


  public double getDistanceFromCameraToGoal(){
    double TARGET_ANGLE_OFFSET = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ANGLE_TO_GOAL = LimelightConstants.LIMELIGHT_MOUNT_ANGLE + TARGET_ANGLE_OFFSET;
    
    double distanceCameraToGoalInches = (LimelightConstants.GOAL_HEIGHT - LimelightConstants.LIMELIGHT_HEIGHT)/Math.tan(ANGLE_TO_GOAL);
    return(distanceCameraToGoalInches);
  } 

  // public static Vision getInstance(){
  //   if(instance == null){
  //     instance = new Vision();
  //   }
  // }

  // public double getDistance(Targets target){
  //   if(!targetFound()){
  //     return false;
  //   }
  //   double angleInRadians = ((SmartDashboard.getNumber("limelight angle", MOUNT_ANGLE) + ty.getDouble(0.0)) * Math.PI) / 180; //maybe switch this to use `Math.toRadians()`
  //       double distance = target.height/Math.tan(angleInRadians);
  //       return distance;
  // }


  public void AlignWithTool(){
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}