package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

//import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

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
    int hookMove = m_Joystick.getPOV();
    double armMove = m_Joystick.getY();

    if (hookMove==0 || hookMove==45 || hookMove==315){
      m_Climber.extendHook();
    }
    else if (hookMove==180 || hookMove==135 || hookMove==215){
      m_Climber.pullHook();
    }
    else {
      m_Climber.stopHook();
    }

    if (armMove < -0.3){
      m_Climber.reachArmBack();
    }
    else if (armMove > 0.3){
      m_Climber.pullArmForward();
    }
    else {
      m_Climber.stopArm();
    }
  }

}
