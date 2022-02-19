package frc.robot.commands;

//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;
//import frc.robot.subsystems.SwerveModuleMK3;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrivetrain drivetrain;
  //private final XboxController leftCcontroller;
  private final Joystick FlightController;
  // private final Joystick rightController;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(6);

  public SwerveDriveCommand(SwerveDrivetrain drivetrain, Joystick controller) {
  //public SwerveDriveCommand(SwerveDrivetrain drivetrain, XboxController controller) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.FlightController = controller;
    // this.rightController = controller2;
  }

  @Override
  public void execute() {
    // SmartDashboard.putNumber("X channel", FlightController.getX());
    // SmartDashboard.putNumber("Y channel", FlightController.getY());
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double X = -FlightController.getX();
    if (Math.abs(X)<0.05){
      X = 0;
    }
    double Y = FlightController.getY();
    if (Math.abs(Y)<0.05){
      Y = 0;
    }
    double Z = FlightController.getZ();
    if (Math.abs(Z)<0.05){
      Z = 0;
    }
    final var xSpeed =
      xspeedLimiter.calculate(Y)
        * DriveConstants.kMaxSpeedMetersPerSecond;
    
   // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
      yspeedLimiter.calculate(X)
        * DriveConstants.kMaxSpeedMetersPerSecond;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
       rotLimiter.calculate(Z)
         * DriveConstants.kMaxSpeedMetersPerSecond;
    // final var rot =
    //     -rotLimiter.calculate(0) //leftController.getZ())
    //       * SwerveDrivetrain.kMaxAngularSpeed;
    boolean calibrate = false; //controller.getLeftBumper();
//SmartDashboard.putBoolean("calibrate", calibrate);

    drivetrain.drive(xSpeed, ySpeed, rot, true, calibrate);
   
  }

}
