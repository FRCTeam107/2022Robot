package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModuleMK3;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrivetrain drivetrain;
  //private final XboxController leftCcontroller;
  private final Joystick leftController;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(6);

  public SwerveDriveCommand(SwerveDrivetrain drivetrain, Joystick controller) {
  //public SwerveDriveCommand(SwerveDrivetrain drivetrain, XboxController controller) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.leftController = controller;
  }

  @Override
  public void execute() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
      -xspeedLimiter.calculate(leftController.getY())
        * SwerveDrivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
      -yspeedLimiter.calculate(leftController.getX())
        * SwerveDrivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    // final var rot =
    //   -rotLimiter.calculate(controller.getRightX())
    //     * SwerveDrivetrain.kMaxAngularSpeed;
    final var rot =
        -rotLimiter.calculate(0) //leftController.getZ())
          * SwerveDrivetrain.kMaxAngularSpeed;
    boolean calibrate = false; //controller.getLeftBumper();
//SmartDashboard.putBoolean("calibrate", calibrate);

    drivetrain.drive(xSpeed, ySpeed, rot, true, calibrate);
    
  }

}