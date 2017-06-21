package org.usfirst.frc.team3322;/*
 * Created by 3322 - Programming 3 on 3/2/2017.
 */

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

public class robot extends IterativeRobot {
    RobotDrive driveTrain;
    DriverStation ds = DriverStation.getInstance();
    static OI xbox;
    static climber climbControl;
    public static AHRS navx = new AHRS(SerialPort.Port.kMXP);
    String AutonTime;
    Servo lServo = new Servo(8);
    Servo rServo = new Servo(9);
    double heading;
    double lTriggerValue;
    double previousThrottle = 0,
            previousTurn = 0,
            maxTurnDelta = .1,
            maxThrottleDelta = .1,
            turnValue,
            throttleValue,
            currentTurn,
            currentThrottle;
    DriverStation.Alliance color;
    boolean isRed;
    static I2C Arduino = new I2C(I2C.Port.kOnboard, 4);
    Talon wapomatic;
    CANTalon dumper;
    public enum autonModes{
        DUMP,
        BACKUP1,
        TURN1,
        BACKUP2,
        TURN2,
        DONE
    }
    autonModes autonMode;
    long targetDuration;
    long switchTime;
    long autonStart;
    double targetHeading;
    double autonSpeed;
    double autonTurn;
    double hopBackSpeed;
    double hopBackTime;
    double maxWap;
    double clampPow;
    // makes variables editable via Git
    Preferences prefs;
    //Ultrasound sensors
    AnalogInput ultraL = new AnalogInput(0);
    AnalogInput ultraR = new AnalogInput(1);

    public void robotInit() {
        //ingestion mechanism
        wapomatic = new Talon(6);
        driveTrain = new RobotDrive(1, 0, 3, 2);
        //ball dumper
        dumper = new CANTalon(4);
        dumper.enableLimitSwitch(true, true);
        //controller
        xbox = new OI();
        CameraServer.getInstance().startAutomaticCapture();
        climbControl = new climber();
        //gyros
        //gyroSPI = new ADXRS450_Gyro();
        //gyroSPI.calibrate();
        lServo.setAngle(100);
        rServo.setAngle(100);
        ledMode("ENABLED");

    }

    public void autonomousInit() {
        // creates variables editable via SmartDashboard
        prefs = Preferences.getInstance();
        autonSpeed = prefs.getDouble("autonSpeed", 0.5);
        autonTurn = prefs.getDouble("autonTurn", 0.25);
        SmartDashboard.putNumber("autonSpeed", autonSpeed);
        SmartDashboard.putNumber("autonTurn", autonTurn);
        // Alliance color
        color = ds.getAlliance();
        isRed = (color == DriverStation.Alliance.Red);
        SmartDashboard.putBoolean("isRed", isRed);
        //gyroSPI.reset();
        navx.reset();
        lServo.setAngle(0);
        rServo.setAngle(25);
        autonStart = System.currentTimeMillis();
        ledMode("ENABLED");
        // resets auton
        autonMode = autonModes.DUMP;

    }

    public void ledMode(String mode) {
        // For contact between RIO and Arduino
        char[] CharArray = mode.toCharArray();
        byte[] WriteData = new byte[CharArray.length];
        for (int i = 0; i < CharArray.length; i++) {
            WriteData[i] = (byte) CharArray[i];
        }
        Arduino.transaction(WriteData, WriteData.length, null, 0);
    }

    public void autonomousPeriodic() {
        // auton time
        AutonTime = String.valueOf(System.currentTimeMillis());
        SmartDashboard.putString("AutonTime",AutonTime);
        SmartDashboard.putString("autonMode", String.valueOf(autonMode));
        // Auton plan: Dump and Run!
        switch (autonMode){
            case DUMP:
                switchTime = System.currentTimeMillis();
                targetDuration = 5000;
                ledMode("DUMP");
                while(System.currentTimeMillis() - switchTime < targetDuration){
                    dumper.set(1);
            }
                dumper.set(-1);
                autonMode = autonModes.BACKUP1;
                break;
            case BACKUP1:
                switchTime = System.currentTimeMillis();
                targetDuration = 5000;
                if (isRed){
                    ledMode("REDBACK");
                } else ledMode("BLUBACK");
                while(System.currentTimeMillis() - switchTime < targetDuration){
                    driveTrain.arcadeDrive(-autonSpeed, 0);
                }
                autonMode = autonModes.TURN1;
                break;
            case TURN1:
                switchTime = System.currentTimeMillis();
                targetDuration = 3000;
                if (isRed) {
                    targetHeading = navx.getYaw() + 45;
                    ledMode("REDIDLE");
                } else {
                    targetHeading = navx.getYaw() - 45;
                    ledMode("BLUIDLE");
                }
                SmartDashboard.putNumber("targetHeading", targetHeading);
                while((Math.abs(targetHeading - navx.getYaw()) > 0.5) && (System.currentTimeMillis() - switchTime < targetDuration)){
                    if (isRed) driveTrain.arcadeDrive(0, -autonTurn);
                   else  driveTrain.arcadeDrive(0, autonTurn);
                   SmartDashboard.putNumber("heading", navx.getYaw());
                }
                autonMode = autonModes.BACKUP2;
                break;
            case BACKUP2:
                switchTime = System.currentTimeMillis();
                targetDuration = 3000;
                if (isRed) {
                    ledMode("REDBACK");
                } else ledMode("BLUBACK");
                while(System.currentTimeMillis() - switchTime < targetDuration){
                    driveTrain.arcadeDrive(-autonSpeed, 0);
                }
                autonMode = autonModes.TURN2;
                break;
            case TURN2:
                switchTime = System.currentTimeMillis();
                targetDuration = 3000;
                if (isRed) {
                    targetHeading = navx.getYaw() - 80;
                } else targetHeading = navx.getYaw() + 80;
                SmartDashboard.putNumber("targetHeading", targetHeading);
                while((Math.abs(targetHeading - navx.getYaw()) > 0.5) && (System.currentTimeMillis() - switchTime < targetDuration)){
                    if (isRed) driveTrain.arcadeDrive(0, autonTurn);
                    else  driveTrain.arcadeDrive(0, -autonTurn);
                    SmartDashboard.putNumber("heading", navx.getYaw());
                }
                autonMode = autonModes.DONE;
                break;
            case DONE:
                break;
        }
    }

    public void disabledInit() {
        ledMode("DISABLED");
    }

    public void disabledPeriodic() {
        lServo.setAngle(100);
        rServo.setAngle(100);
    }

    public void teleopInit() {
        prefs = Preferences.getInstance();
        hopBackSpeed = prefs.getDouble("hopBackSpeed", 0.2);
        hopBackTime = prefs.getDouble("hopBackTime", 100);
        maxWap = prefs.getDouble("maxWap", 0.9);
        SmartDashboard.putNumber("maxWap", maxWap);
        SmartDashboard.putNumber("hopBackSpeed", hopBackSpeed);
        SmartDashboard.putNumber("hopBackTime", hopBackTime);
        ultraL.setGlobalSampleRate(1000);
        ultraR.setGlobalSampleRate(1000);
        ultraL.setOversampleBits(10);
        ultraR.setOversampleBits(10);
        ultraL.setAverageBits(5);
        ultraR.setAverageBits(5);
        lServo.setAngle(0);
        rServo.setAngle(25);
        navx.reset();
    }

    public void teleopPeriodic() {
        // analogWap = lTrigger
        // wapVomit = lBumper
        // drive = lStickY
        // turn = rStickX
        // dump = holdAButton
        // hopBack = bButton
        // forceClimb = xButton
        // invertDrive = rBumper
        // climb = yButton
        // taunt = dpad
            while (isOperatorControl() && isEnabled()) {
            int distL;
            int distR;
            heading = navx.getYaw();
            distL = ultraL.getValue();
            distR = ultraR.getValue();
            SmartDashboard.putNumber("heading", heading);
            SmartDashboard.putNumber("distL", distL);
            SmartDashboard.putNumber("distR", distR);
            if(isRed){
                ledMode("REDFWD");
            } else ledMode("BLUFWD");
            if (xbox.heldDown(OI.START)){
                lServo.setAngle(0);
                rServo.setAngle(25);
            }
            if (xbox.heldDown(OI.BACK)){
                lServo.setAngle(100);
                rServo.setAngle(100);
            }
            if(xbox.pressedOnce(OI.BBUTTON)) {
                //hopBack
                if (isRed = true){
                    ledMode("REDIDLE");
                } else ledMode("BLUIDLE");
                switchTime = System.currentTimeMillis();
                while (System.currentTimeMillis() - switchTime < hopBackTime){
                    driveTrain.arcadeDrive(hopBackSpeed, 0);
                }
                switchTime = System.currentTimeMillis();
                while (System.currentTimeMillis() - switchTime < hopBackTime){
                    driveTrain.arcadeDrive((hopBackSpeed * -1.2), 0);
                }
            }
            clamp();
            currentTurn = currentTurn * -1;
            if((xbox.isToggled(OI.DPADVERT)) || (xbox.isToggled(OI.DPADHORIZ))){
                ledMode("TAUNT");
            }
            if(xbox.isToggled(OI.RBUMPER)) {
                currentThrottle = currentThrottle * -1;
                if(isRed){
                    ledMode("REDBACK");
                } else ledMode("BLUBACK");
            }
            if(xbox.heldDown(OI.ABUTTON)) {
                dumper.set(1);
                ledMode("DUMP");
            } else dumper.set(-1);
            driveTrain.arcadeDrive(-currentThrottle,currentTurn);
            climbControl.climb(OI.YBUTTON, OI.XBUTTON);
            if (xbox.heldDown(OI.XBUTTON) || (xbox.isToggled(OI.YBUTTON))){
                    ledMode("UP");
                }
            lTriggerValue = Math.abs(xbox.getAxis(2));
            SmartDashboard.putNumber("LTriggerValue", lTriggerValue);
            if (xbox.isToggled(OI.LBUMPER)) {
                if (lTriggerValue < maxWap) {
                    wapomatic.set(lTriggerValue);
                } else wapomatic.set(maxWap);
            } else if (lTriggerValue < maxWap) {
                wapomatic.set(-lTriggerValue);
            } else wapomatic.set(-maxWap);
            SmartDashboard.putNumber("heading", heading);
        }

    }
    private void clamp(){
        clampPow = prefs.getDouble("clampPow", 2);
        SmartDashboard.putNumber("clampPow", clampPow);
        currentThrottle = xbox.getFineAxis(OI.L_YAXIS, clampPow);
        currentTurn = xbox.getFineAxis(OI.R_XAXIS, clampPow);

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
