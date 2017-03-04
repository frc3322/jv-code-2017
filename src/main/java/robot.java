/*
 * Created by 3322 - Programming 3 on 3/2/2017.
 */

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class robot extends SampleRobot {

    RobotDrive myDrive;
    Joystick left, right;
    String AutonTime;

    public void robotInit() {
        myDrive = new RobotDrive(2, 3, 1, 0);
        left = new Joystick(1);
        right = new Joystick(2);
        SmartDashboard.putString("Hello_World","Hi!");
    }

    public void autonomousPeriodic() {
        AutonTime = String.valueOf(System.currentTimeMillis());
        SmartDashboard.putString("AutonTime",AutonTime);
    }

    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
            myDrive.tankDrive(left, right);
            Timer.delay(0.01);
        }
    }

}
