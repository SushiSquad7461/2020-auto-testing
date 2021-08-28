package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class OI {
    public static double getTriggers(XboxController controller) {
        double rightTrigger = controller.getTriggerAxis(Hand.kRight);
        double leftTrigger = controller.getTriggerAxis(Hand.kLeft);
    
        return Math.pow(rightTrigger-leftTrigger, 3);
    } 

    public static double getLeftJoystickAxis(XboxController controller) {
        return Math.pow(controller.getX(GenericHID.Hand.kLeft), 3);
    }
}
