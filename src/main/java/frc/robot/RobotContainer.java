// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerJoystick;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Shoot;
//import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Joystick m_leftJoystick, m_rightJoystick, m_controllerJoystick;
  private final SwerveDrivetrain m_Drivetrain;
  private final Shooter m_shooter;
  // private final Compressor m_compressor;

  //private final XboxController controller = new XboxController(0);
  //private final Joystick controller = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_leftJoystick = new Joystick(Constants.UsbPorts.LEFT_STICK);
    m_rightJoystick = new Joystick(Constants.UsbPorts.RIGHT_STICK);
    m_controllerJoystick = new Joystick(Constants.UsbPorts.CONTROLLER_STICK);

    m_Drivetrain  = new SwerveDrivetrain();
    m_shooter = new Shooter ();

    m_Drivetrain.setDefaultCommand(new SwerveDriveCommand(m_Drivetrain, m_leftJoystick, m_rightJoystick));
    
    configureButtonBindings();


  }

  private void configureButtonBindings() {
    // temporary buttons for debugging
    //new JoystickButton(m_rightJoystick, 6).whenPressed(new PivotToAngle(-45, 0.5, m_drivetrain, m_gyro));
    //  new JoystickButton(m_leftJoystick, 1).whenPressed(new DriveDistance(-0.4, 50, 180, m_drivetrain, m_gyro, 60));
    //new JoystickButton(m_leftJoystick, 15).whenPressed(new DriveDistance(0.4, 50, 0, m_drivetrain, m_gyro, 60));
    //new JoystickButton(m_rightJoystick, 9).whenPressed(new PivotToAngle(45, 0.5, m_drivetrain, m_gyro));

    //new JoystickButton(m_leftJoystick,8).whenReleased(m_drivetrain::resetEncoders);

    
    // DRIVER'S LEFT JOYSTICK BUTTONS

    // WARNING:  This code could mess up the turret if the Re-Sync turret is pressed during a match!!
    // to sync turret, 2 buttons are required, both MANUAL_TURRET and SYNC_TURRET, these are on separate joysticks
    
    // robot facing 180 degrees (pick-up facing away from goal)
   // new JoystickButton(m_leftJoystick, LeftJoystick.SYNC_TURRET).whenPressed(new SyncTurretWithGyro(180, m_turret, m_gyro, () -> true ));

    // DRIVER'S RIGHT JOYSTICK BUTTONS
    //new JoystickButton(m_rightJoystick, RightJoystick.TOGGLE_LIMELIGHT).whenPressed(m_limelight::ToggleVisionProcessing, m_limelight);
    new JoystickButton(m_controllerJoystick, ControllerJoystick.SHOOT).whileHeld(
                new Shoot(m_shooter, 
                () -> m_controllerJoystick.getRawButton(ControllerJoystick.FORCE_READY) ));

    //new JoystickButton(m_leftJoystick, 15).whenPressed(m_LEDs::LigthEmUp);
    
    // CONTROLLER'S JOYSTICK BUTTONS
     // JoystickButton btnManualOverride = new JoystickButton(m_controllerJoystick, ControllerJoystick.MANUAL_OVERRIDE);
      //btnManualOverride.whenPressed(m_climber::allowAdditionalMovement);
      // btnManualOverride.whenReleased(m_climber::setToRetractedPosition);

    }


      /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
//SwerveControllerCommand x = new SwerveControllerCommand(trajectory, pose, kinematics, xController, yController, thetaController, outputModuleStates, requirements)

SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_Drivetrain::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_Drivetrain::setModuleStates,
            m_Drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    m_Drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_Drivetrain.drive(0, 0, 0, false, false));
  }

}
