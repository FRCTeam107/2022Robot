// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerJoystick;
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
  private final SwerveDrivetrain m_dDrivetrain;
  private final Shooter m_shooter;
  // private final Compressor m_compressor;

  //private final XboxController controller = new XboxController(0);
  //private final Joystick controller = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_leftJoystick = new Joystick(Constants.UsbPorts.LEFT_STICK);
    m_rightJoystick = new Joystick(Constants.UsbPorts.RIGHT_STICK);
    m_controllerJoystick = new Joystick(Constants.UsbPorts.CONTROLLER_STICK);

    m_dDrivetrain  = new SwerveDrivetrain();
    m_shooter = new Shooter ();

    m_dDrivetrain.setDefaultCommand(new SwerveDriveCommand(m_dDrivetrain, m_leftJoystick, m_rightJoystick));
    
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

}
