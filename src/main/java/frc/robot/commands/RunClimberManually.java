package frc.robot.commands;

//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SwerveDrivetrain;
//import frc.robot.subsystems.SwerveModuleMK3;

public class RunClimberManually extends CommandBase {

  private final Climber m_Climber;
  //private final XboxController leftCcontroller;
  private final Joystick m_Joystick;
  // private final Joystick rightController;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.

  public RunClimberManually(Climber _climber, Joystick controller) {
  //public SwerveDriveCommand(SwerveDrivetrain drivetrain, XboxController controller) {
    m_Climber = _climber;
    m_Joystick = controller;
    addRequirements(m_Climber);
  }

  @Override
  public void execute() {
    double X = -    m_Joystick.getX();
    double Y = m_Joystick.getY();
    double Z = m_Joystick.getZ();
  }

}
