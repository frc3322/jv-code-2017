package org.usfirst.frc.team3322;/*
 * Created by 3322 - Programming 3 on 3/2/2017.
 */

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class robot extends SampleRobot {

    RobotDrive myDrive;
    Joystick driveStick;
    String AutonTime;

    public void robotInit() {
        myDrive = new RobotDrive(2, 3, 1, 0);
        driveStick = new Joystick(0);
        SmartDashboard.putString("Hello_World","Hi!");
    }

    public void autonomousInit() {
        SmartDashboard.putNumber("StartPosInCode",3322);
        SmartDashboard.putNumber("auton", 2);
        SmartDashboard.putBoolean("enabled", true);
    }

    public void autonomousPeriodic() {
        AutonTime = String.valueOf(System.currentTimeMillis());
        SmartDashboard.putString("AutonTime",AutonTime);
    }

    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
                myDrive.arcadeDrive(driveStick);
            Timer.delay(0.01);
        }
    }

}
