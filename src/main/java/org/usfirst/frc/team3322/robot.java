package org.usfirst.frc.team3322;/*
 * Created by 3322 - Programming 3 on 3/2/2017.
 */

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class robot extends IterativeRobot {

    RobotDrive myDrive;
    Joystick driveStick;
    String AutonTime;

    public void robotInit() {
        myDrive = new RobotDrive(2, 3, 1, 0);
        driveStick = new Joystick(0);
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

    public void disabledInit() {
        SmartDashboard.putNumber("x_length", 100);
        SmartDashboard.putNumber("y_length", 132);
    }

    public void disabledPeriodic() {
        SmartDashboard.putBoolean("auton_ready",false);
        SmartDashboard.putNumber("StartPosInCode",42);
        SmartDashboard.putBoolean("enabled",false);
    }

    public void teleopInit() {
        SmartDashboard.putNumber("teleop",0);
        SmartDashboard.putNumber("auton",0);
        SmartDashboard.putBoolean("enabled", true);
    }

    public void teleopPeriodic() {
        while (isOperatorControl() && isEnabled()) {
                myDrive.arcadeDrive(driveStick);
            Timer.delay(0.01);
        }
    }

}
