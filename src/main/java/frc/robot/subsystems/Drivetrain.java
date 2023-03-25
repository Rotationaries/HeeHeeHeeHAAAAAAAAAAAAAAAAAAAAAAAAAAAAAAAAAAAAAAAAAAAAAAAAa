// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Autonomous.Balance;

public class Drivetrain extends SubsystemBase {

  public static CANSparkMax leftMotor1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  public static CANSparkMax leftMotor2 = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);

  MotorControllerGroup m_leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);


  // The motors on the right side of the drive.
  public static CANSparkMax rightMotor1 = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  public static CANSparkMax rightMotor2 = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

  MotorControllerGroup m_rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);


  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  private final RelativeEncoder m_leftEncoder1 = leftMotor1.getEncoder();
  private final RelativeEncoder m_leftEncoder2 = leftMotor2.getEncoder();
  private final RelativeEncoder m_rightEncoder1 = rightMotor1.getEncoder();
  private final RelativeEncoder m_rightEncoder2 = rightMotor2.getEncoder();

  // The gyro sensor
  private final AHRS ahrs;

  // Odometry class for tracking robot pose
  private static DifferentialDriveOdometry m_odometry;

  // These classes help us simulate our drivetrain
  public DifferentialDrivetrainSim m_drivetrainSimulator;

  // public REVPhysicsSim m_revPhysicsSim = new REVPhysicsSim();
  // private SimDeviceSim m_leftEncoder1Sim;
  // private SimDeviceSim m_rightEncoder1Sim;
  // private SimDeviceSim m_leftEncoder2Sim;
  // private SimDeviceSim m_rightEncoder2Sim;
  // The Field2d class shows the field in the sim GUI
  private Field2d m_fieldSim = new Field2d();

  private static double rate;
  private static double speed;
  private static double turn;

  public static XboxController controller = new XboxController(0);

  public double m_currentForwardVelocity = 0;
  public double m_currentTurningVelocity = 0;

  /** Creates a new ExampleSubsystem. */

  public Drivetrain() {
    m_rightMotors.setInverted(true);

    /*leftMotor1.setOpenLoopRampRate(2);
    leftMotor2.setOpenLoopRampRate(2);
    rightMotor1.setOpenLoopRampRate(2);
    rightMotor2.setOpenLoopRampRate(2);*/

    // m_leftEncoder1.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    
    ahrs = new AHRS(SPI.Port.kMXP);

    // resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), 0, 0);
    if (RobotBase.isSimulation()) { // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      m_drivetrainSimulator =
          new DifferentialDrivetrainSim(
              DriveConstants.kDrivetrainPlant,
              DriveConstants.kDriveGearbox,
              DriveConstants.kDriveGearing,
              DriveConstants.kTrackwidthMeters,
              DriveConstants.kWheelDiameterMeters / 2.0,
              VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));
      // The encoder and gyro angle sims let us set simulated sensor readings
      // REVPhysicsSim.getInstance().addSparkMax(leftMotor1, DCMotor.getNEO(1));
      // REVPhysicsSim.getInstance().addSparkMax(leftMotor2, DCMotor.getNEO(1));
      // REVPhysicsSim.getInstance().addSparkMax(rightMotor1, DCMotor.getNEO(1));
      // REVPhysicsSim.getInstance().addSparkMax(rightMotor2, DCMotor.getNEO(1));
      // the Field2d class lets us visualize our robot in the simulation GUI.
      SmartDashboard.putData("Field", m_fieldSim);
    }
  }

  public CANSparkMax getMotor1(){
    return leftMotor1;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        m_leftEncoder1.getPosition(),
        m_rightEncoder1.getPosition());
    m_fieldSim.setRobotPose(getPose());
    SmartDashboard.putNumber("ControllerY", -controller.getLeftY());
    SmartDashboard.putNumber("ControllerX", -controller.getRightX());
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_drivetrainSimulator.setInputs(
        m_leftMotors.get() * RobotController.getBatteryVoltage(),
        m_rightMotors.get() * RobotController.getBatteryVoltage());
    m_drivetrainSimulator.update(0.020);
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getLeftEncoderVelocity() {
    return (m_leftEncoder1.getVelocity() + m_leftEncoder2.getVelocity()) / 2;
  }

  public double getRightEncoderVelocity() {
    return (m_rightEncoder1.getVelocity() + m_rightEncoder2.getVelocity()) / 2;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  // public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  //   return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  // }

  public void resetOdometry(Pose2d pose) {
    // resetEncoders();
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), m_leftEncoder1.getPosition(), m_rightEncoder1.getPosition(), pose);
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
    System.out.println(fwd);
    System.out.println(rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    var batteryVoltage = RobotController.getBatteryVoltage();
    if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts)) > batteryVoltage) {
      leftVolts *= batteryVoltage / 12.0;
      rightVolts *= batteryVoltage / 12.0;
    }
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  // public void resetEncoders() {
  //   m_leftEncoder.reset();
  //   m_rightEncoder.reset();
  // }

  // public int getLeftEncoderCount() {
  //   return m_leftEncoder.get();
  // }

  // public int getRightEncoderCount() {
  //   return m_rightEncoder.get();
  // }

  // public double getLeftDistanceInch() {
  //   return m_leftEncoder.getDistance();
  // }

  // public double getRightDistanceInch() {
  //   return m_rightEncoder.getDistance();
  // }

  // public double getAverageDistanceInch() {
  //   return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  // }

  // public double getLeftVelocity() {
  //   return m_leftEncoder.getRate();
  // }

  // public double getRightVelocity() {
  //   return m_rightEncoder.getRate();
  // }

  // public double getAverageEncoderDistance() {
  //   return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  // }

  // public Encoder getLeftEncoder() {
  //   return m_leftEncoder;
  // }

  // public Encoder getRightEncoder() {
  //   return m_rightEncoder;
  // }

  public double getLeftEncoder1Rotations() {
    return m_leftEncoder1.getPosition();
  }

  public double getLeftEncoder2Rotations() {
    return m_leftEncoder1.getPosition();
  }

  public double getRightEncoder1Rotations() {
    return m_leftEncoder1.getPosition();
  }

  public double getRightEncoder2Rotations() {
    return m_leftEncoder1.getPosition();
  }

  public double getAverageEncoderRotations() {
    return (getLeftEncoder1Rotations()+getLeftEncoder2Rotations()+getRightEncoder1Rotations()+getRightEncoder2Rotations()) / 4;
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    ahrs.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(ahrs.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  
  public double getRoll(){
    return ahrs.getRoll();
  }

  public double getPitch(){
    return ahrs.getPitch();
  }

  public Command getBalance(){
    return Commands.sequence(new Balance());
  }

  public void controllerMovement(XboxController controller){

    // rate = (0.5 * -controller.getRawAxis(3)) + 0.5;

    // speed = controller.getLeftY();

    // turn = -controller.getRightX();

    // double left = speed + turn;

    // double right = speed - turn;

    // leftMotor1.set(left);

    // leftMotor2.set(-left);

    // rightMotor1.set(right);

    // rightMotor2.set(-right);

    double targetForwardVelocity = -controller.getLeftY();

    double targetTurningVelocity = -controller.getRightX();



    // m_currentForwardVelocity

    // m_currentTurningVelocity

  

    double delta_forward = (targetForwardVelocity > m_currentForwardVelocity) ? DriveConstants.DELTA_FORWARD : -DriveConstants.DELTA_FORWARD;

    double delta_turning = (targetTurningVelocity > m_currentTurningVelocity) ? DriveConstants.DELTA_TURNING : -DriveConstants.DELTA_TURNING;

    m_currentForwardVelocity = m_currentForwardVelocity + delta_forward /*multiply by period*/ ;

    m_currentTurningVelocity = m_currentTurningVelocity + delta_turning; 

    

//    m_drive.arcadeDrive(-controller.getLeftY(), -controller.getRightX()); 

    m_drive.arcadeDrive(m_currentForwardVelocity, m_currentTurningVelocity);



    // leftMotor1.set(0.5);

    // leftMotor2.set(0.5);

    // rightMotor1.set(0.6);

    // rightMotor2.set(-0.5);





  }
}