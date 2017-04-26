package org.usfirst.frc.team3322;/*
 * Created by 3322 - Programming 3 on 3/2/2017.
 */

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class robot extends IterativeRobot {
    RobotDrive myDrive;
    DriverStation ds = DriverStation.getInstance();
    static OI xbox;
    static climber climbcontrol;
    String AutonTime;
    boolean drivingStraight = false;
    boolean climbing;
    boolean dumping;
    double lTriggerValue;
    double xLength,
            yLength,
            driveStraightAngle,
            previousThrottle = 0,
            previousTurn = 0,
            maxTurnDelta = .05,
            maxThrottleDelta = .05,
            turnValue,
            throttleValue,
            currentTurn,
            currentThrottle;
    boolean isRed;
    static I2C Arduino = new I2C(I2C.Port.kOnboard, 4);
    Talon wapomatic;

    public void robotInit() {
        wapomatic = new Talon(6);
        myDrive = new RobotDrive(2, 3, 1, 0);
       // driveStick = new Joystick(0);
        xbox = new OI();
        CameraServer.getInstance().startAutomaticCapture();
        DriverStation.Alliance color;
        color = ds.getAlliance();
        isRed = (color == DriverStation.Alliance.Red);
        SmartDashboard.putBoolean("isRed", isRed);
        climbcontrol = new climber();
    }

    public void autonomousInit() {
        SmartDashboard.putNumber("StartPosInCode",3322);
        SmartDashboard.putNumber("auton", 2);
        SmartDashboard.putBoolean("enabled", true);
    }

    public void ledMode(String mode) {
        char[] CharArray = mode.toCharArray();
        byte[] WriteData = new byte[CharArray.length];
        for (int i = 0; i < CharArray.length; i++) {
            WriteData[i] = (byte) CharArray[i];
        }
        Arduino.transaction(WriteData, WriteData.length, null, 0);
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
                //myDrive.arcadeDrive(driveStick);
    //        Timer.delay(0.01);
      //      currentThrottle = xbox.getFineAxis(OI.L_YAXIS, 2);
        //    currentTurn = xbox.getFineAxis(OI.R_XAXIS, 2);
            clamp();
            if(xbox.isToggled(OI.RBUMPER)) {
                currentThrottle = currentThrottle * -1;
                currentTurn = currentTurn * -1;
            }
            if(xbox.isToggled(OI.YBUTTON)) {
                if (climbing == false){
                    climbing = true;
                } else if (climbing == true){
                    climbing = false;
                }
                }
           // SmartDashboard.putNumber("Right trigger", rTriggerValue);
            myDrive.arcadeDrive(-currentThrottle,currentTurn);
            if(xbox.heldDown(OI.ABUTTON))
                ledMode("on");
            if(xbox.heldDown(OI.BBUTTON))
                ledMode("off");
            if(climbing == true) {
                ledMode("up");
                SmartDashboard.putBoolean("climb", true);
            }
            if(dumping == true) {
                ledMode("dump");
                SmartDashboard.putBoolean("dump", true);
            } else {
                ledMode("normal");
                SmartDashboard.putBoolean("dump", false);
            }
          //myDrive.arcadeDrive(-throttleValue, turnValue);
            //drivingStraight = false;
            climbcontrol.climb(OI.YBUTTON, OI.BBUTTON);
            lTriggerValue = Math.abs(xbox.getAxis(2));
            SmartDashboard.putNumber("LTriggerValue", lTriggerValue);
            wapomatic.set(lTriggerValue);
        }
    }
    private void clamp(){
        currentThrottle = xbox.getFineAxis(OI.L_YAXIS, 3);
        currentTurn = xbox.getFineAxis(OI.R_XAXIS, 3);

        double deltaTurn = currentTurn - previousTurn;
        double deltaThrottle = currentThrottle - previousThrottle;

        if(Math.abs(deltaTurn) > maxTurnDelta && (previousTurn / deltaTurn) > 0){
            turnValue = previousTurn + ((deltaTurn < 0)? -maxTurnDelta : maxTurnDelta);
        } else {
            turnValue = currentTurn;
        }

        if (Math.abs(deltaThrottle) > maxThrottleDelta && (previousThrottle / deltaThrottle) > 0) {
            throttleValue = previousThrottle + ((deltaThrottle < 0)? -maxThrottleDelta : maxThrottleDelta);
        } else {
            throttleValue = currentThrottle;
        }

        previousThrottle = throttleValue;
        previousTurn = turnValue;

        SmartDashboard.putNumber("turn_value", turnValue);
        SmartDashboard.putNumber("joystick", currentTurn);
    }
}
