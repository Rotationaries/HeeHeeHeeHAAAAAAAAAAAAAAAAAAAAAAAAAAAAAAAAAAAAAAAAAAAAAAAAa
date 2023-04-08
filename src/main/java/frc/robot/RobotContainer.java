// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autonomous.FollowingPath;
import frc.robot.commands.Autonomous.GoForward;
//import frc.robot.commands.Autonomous.FollowingPath;
//import frc.robot.commands.Autonomous.CreatingPaths;
//import frc.robot.commands.Autonomous.FollowingPath;
import frc.robot.commands.Drive.Drive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cascade;
import frc.robot.subsystems.Drivetrain;
//import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Intake;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //private Intake intake = new Intake();
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_robotDrive = new Drivetrain();
  private final FollowingPath pathFollower = new FollowingPath(m_robotDrive);
  private PathPlannerTrajectory path = new PathPlannerTrajectory();
  //private Map<String, Command> eventMap = new HashMap<>();
  public PIDController x = new PIDController(0.2, 0, 0);
  public PIDController y = new PIDController(0.2,0,0);
  private List<PathPlannerTrajectory> alist = new ArrayList<>();
  private List<Command> clist = new ArrayList<>();
  private String fileName;
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private Map<String, Command> eventMap = new HashMap<>();
  private Cascade cascade = new Cascade();
  private Arm arm = new Arm();

  private Command m_directDock2 = new SequentialCommandGroup(
    new GoForward(m_robotDrive, 4));
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Containssubsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    alist = PathPlanner.loadPathGroup(fileName, AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_robotDrive::exampleCondition)
        .onTrue(new Drive(m_robotDrive));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //.y().whileTrue(intake.getAlign()); //Random subsystem to run in
    m_driverController.b().whileTrue(m_robotDrive.getBalance());
  }

  public Drivetrain getRobotDrive() {
    return m_robotDrive;
  }

  public void zeroAllOutputs() {
    m_robotDrive.tankDriveVolts(0, 0);
    
  }

  public void configureAuto() {
    List<PathPlannerTrajectory> paths1 = PathPlanner.loadPathGroup("DirectDock-M", AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    System.out.println("Paths 1: "+paths1.size());
    /*List<PathPlannerTrajectory> paths2 = PathPlanner.loadPathGroup("1CO - BlueB", AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    System.out.println("Paths 2: "+paths2.size());
    List<PathPlannerTrajectory> paths3 = PathPlanner.loadPathGroup("1CO - BlueM", AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    System.out.println("Paths 3: "+paths3.size());
    List<PathPlannerTrajectory> paths4 = PathPlanner.loadPathGroup("1CO - BlueT", AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    System.out.println("Paths 4: "+paths4.size());
    List<PathPlannerTrajectory> paths5 = PathPlanner.loadPathGroup("1CO - RedB", AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    System.out.println("Paths 5: "+paths5.size());
    List<PathPlannerTrajectory> paths6 = PathPlanner.loadPathGroup("1CO - RedM", AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    System.out.println("Paths 6: "+paths6.size());
    List<PathPlannerTrajectory> paths7 = PathPlanner.loadPathGroup("1CO - RedT", AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    System.out.println("Paths 7: "+paths7.size());*/

    Command m_directDock = new SequentialCommandGroup(
      new GoForward(m_robotDrive, 4)
      //new FollowPathWithEvents(new FollowingPath(m_robotDrive, paths1.get(0)), paths1.get(0).getMarkers(), eventMap),
      //new FollowPathWithEvents(new FollowingPath(m_robotDrive, paths1.get(1)), paths1.get(1).getMarkers(), eventMap),
      //new FollowPathWithEvents(new FollowingPath(m_robotDrive, paths1.get(2)), paths1.get(2).getMarkers(), eventMap),
      //new FollowPathWithEvents(new FollowingPath(m_robotDrive, paths1.get(3)), paths1.get(3).getMarkers(), eventMap),
     // new SkyBalance(m_robotDrive)
    );
    Command m_directDockRamsete = pathFollower.autoPath();


    /*Command m_blueB = new SequentialCommandGroup(
      new WaitCommand(4),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths2.get(0)), paths2.get(0).getMarkers(), eventMap),
      new InstantCommand(() -> { cascade.autoCascadeDrive(CascadeConstants.kstage1);}, cascade),
      new InstantCommand(() -> { arm.armPID(-45);}, arm),
    //  new InstantCommand(() -> { intake.pullIn();}, intake),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths2.get(1)), paths2.get(1).getMarkers(), eventMap),
     // new InstantCommand(() -> { intake.eject(); }, intake),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths2.get(2)), paths2.get(2).getMarkers(), eventMap),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths2.get(3)), paths2.get(3).getMarkers(), eventMap),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths2.get(4)), paths2.get(4).getMarkers(), eventMap),
      new SkyBalance(m_robotDrive)
    );

    Command m_blueM = new SequentialCommandGroup(
      new WaitCommand(4),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths3.get(0)), paths3.get(0).getMarkers(), eventMap),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths3.get(1)), paths3.get(1).getMarkers(), eventMap),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths3.get(2)), paths3.get(2).getMarkers(), eventMap),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths3.get(3)), paths3.get(3).getMarkers(), eventMap),
      new InstantCommand(() -> {cascade.autoCascadeDrive(CascadeConstants.kstage1);}, cascade),
      new InstantCommand(() -> {arm.armPID(-45);}, arm),
     // new InstantCommand(() -> { intake.pullIn();}, intake),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths3.get(4)), paths2.get(4).getMarkers(), eventMap),
     // new InstantCommand(() -> { intake.eject();}, intake),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths3.get(5)), paths2.get(5).getMarkers(), eventMap),
      new SkyBalance(m_robotDrive)
    );

    Command m_blueT = new SequentialCommandGroup(
      new WaitCommand(4),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths4.get(0)), paths4.get(0).getMarkers(), eventMap),
      new InstantCommand(() -> {cascade.autoCascadeDrive(CascadeConstants.kstage1);}, cascade),
      new InstantCommand(() -> {arm.armPID(-45);}, arm),
      //new InstantCommand(() -> { intake.pullIn();}, intake),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths4.get(1)), paths4.get(1).getMarkers(), eventMap),
      //new InstantCommand(() -> { intake.eject();}, intake),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths4.get(2)), paths4.get(2).getMarkers(), eventMap),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths4.get(3)), paths4.get(3).getMarkers(), eventMap),
      new SkyBalance(m_robotDrive)
    );

    Command m_redB = new SequentialCommandGroup(
      new WaitCommand(4),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths5.get(0)), paths5.get(0).getMarkers(), eventMap),
      new InstantCommand(() -> {cascade.autoCascadeDrive(CascadeConstants.kstage1);}, cascade),
      new InstantCommand(() -> {arm.armPID(-45);}, arm),
    //  new InstantCommand(() -> { intake.pullIn();}, intake),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths4.get(1)), paths4.get(1).getMarkers(), eventMap),
    //  new InstantCommand(() -> { intake.eject();}, intake),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths4.get(2)), paths4.get(2).getMarkers(), eventMap),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths4.get(3)), paths4.get(3).getMarkers(), eventMap),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths4.get(4)), paths4.get(4).getMarkers(), eventMap),
      new SkyBalance(m_robotDrive)
    );

    Command m_redM = new SequentialCommandGroup(
      new WaitCommand(4),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths5.get(0)), paths5.get(0).getMarkers(), eventMap),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths5.get(1)), paths5.get(1).getMarkers(), eventMap),
      new InstantCommand(() -> {cascade.autoCascadeDrive(CascadeConstants.kstage1);}, cascade),
      new InstantCommand(() -> {arm.armPID(-45);}, arm),
    //  new InstantCommand(() -> { intake.pullIn();}, intake),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths4.get(2)), paths4.get(2).getMarkers(), eventMap),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths4.get(3)), paths4.get(3).getMarkers(), eventMap),
    //  new InstantCommand(() -> { intake.eject();}, intake),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths4.get(4)), paths4.get(4).getMarkers(), eventMap),
      new SkyBalance(m_robotDrive)
    );

    Command m_redT = new SequentialCommandGroup(
      new WaitCommand(4),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths5.get(0)), paths5.get(0).getMarkers(), eventMap),
      new InstantCommand(() -> {cascade.autoCascadeDrive(CascadeConstants.kstage1);}, cascade),
      new InstantCommand(() -> {arm.armPID(-45);}, arm),
    //  new InstantCommand(() -> { intake.pullIn();}, intake),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths4.get(1)), paths4.get(1).getMarkers(), eventMap),
    //  new InstantCommand(() -> { intake.eject();}, intake),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths4.get(2)), paths4.get(2).getMarkers(), eventMap),
      new FollowPathWithEvents(new FollowingPath(m_robotDrive,paths4.get(3)), paths4.get(3).getMarkers(), eventMap),
      new SkyBalance(m_robotDrive)
    );*/


    // Command scoreCubeDock = new SequentialCommandGroup(

    // )

    Command m_doNothing = new WaitCommand(15);

    m_chooser.setDefaultOption("Direct Dock", m_directDock);
    m_chooser.addOption("Do Nothing", m_doNothing);
    m_chooser.addOption("Middle Direct Dock Ramsete", m_directDockRamsete);
    // m_chooser.addOption("BlueB", m_blueB);
    // m_chooser.addOption("BlueM", m_blueM);
    // m_chooser.addOption("BlueT", m_blueT);
    // m_chooser.addOption("RedB", m_redB);
    // m_chooser.addOption("RedM", m_redM);
    // m_chooser.addOption("RedT", m_redT);

    SmartDashboard.putData("Auto Chooser", m_chooser);
    
    
    /*eventMap.put("turn1", new PrintCommand("passed marker 1"));
    eventMap.put("turn2", new PrintCommand("passed marker 2"));
    eventMap.put("turn3", new PrintCommand("passed marker 3"));
    eventMap.put("turn4", new PrintCommand("passed marker 4"));*/

    // for(int i=0; i<alist.size(); i++) {
    //   clist.add(new FollowPathWithEvents(new FollowingPath(m_robotDrive,alist.get(i), fileName), alist.get(i).getMarkers(), eventMap));
    // }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return new CreatingPaths(m_robotDrive, "DirectDock-M");
    //return m_pathCreator.dockPath();
    //return new DriveDistance(0.5, m_robotDrive);
    //PathPlannerTrajectory traj = PathPlanner.loadPath("1CO1CU-B",new PathConstraints(2, 1.5));
    
    /*PathPlannerTrajectory traj = PathPlanner.loadPath("TestNew", AutoConstants.kMaxSpeedMetersPerSecond, 
      AutoConstants.kMaxAccelerationMetersPerSecondSquared);
      System.out.println(traj);
    PPRamseteCommand command = new PPRamseteCommand(traj, m_robotDrive::getPose, new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
    AutoConstants.feedforward, DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds, 
    x, y, m_robotDrive::tankDriveVolts, m_robotDrive); 
  // ArrayList<PathPoint> pointlist = new ArrayList<PathPoint>(Arrays.asList(new PathPoint(new Translation2d(1.0,1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))));

    
  return command.andThen(() -> m_robotDrive.tankDriveVolts(0,0));*/

  //return m_chooser.getSelected();
    return Commands.sequence(m_directDock2);
  


  /*path = PathPlanner.loadPath("DirectDock-M", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  //paths = PathPlanner.loadPathGroup("1CO", AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  return new SequentialCommandGroup(
    new WaitCommand(4),
    new FollowPathWithEvents(new FollowingPath(m_robotDrive,path), path.getMarkers(), eventMap));*/

  /*return new SequentialCommandGroup(
    new WaitCommand(4),
    clist.split());*/
  }
}

