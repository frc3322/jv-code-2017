package org.usfirst.frc.team3322;

import edu.wpi.first.wpilibj.*;

public class climber {
    PowerDistributionPanel pdp = new PowerDistributionPanel();
    Talon climb_talon_1, climb_talon_2;
    Encoder climbEncoder;
    ClimbState climbStatus;
    // value from 0.00 to 1.00
    double climbRate = 1.0,
            totalCurrent = 0,
            avgCurrent = 0,
            vibration = 0;
    double[] current;
    int timer = 0,
            i = 0;

    public enum ClimbState {
        STOP,
        CLIMB,
        FORCE_CLIMB
    }

    public climber() {
        climb_talon_1 = new Talon(4);
        climb_talon_2 = new Talon(5);
        // climb_talon_1 = new Talon(RobotMap.climbTalon_1);
        // climb_talon_2 = new Talon(RobotMap.climbTalon_2);
        current = new double[] {0,0,0,0,0};
    }

    // Climbs if the climb button is toggled, and can be force-enabled by pressing and holding down the force climb button.
    public void climb(int climbButton, int forceClimbButton) {
        boolean climb = robot.xbox.isToggled(climbButton),
                forceClimb = robot.xbox.heldDown(forceClimbButton);

        avgCurrent();

        if (forceClimb) {
            climbStatus = ClimbState.FORCE_CLIMB;
        } else if (climb) {
            climbStatus = ClimbState.CLIMB;
        } else {
            climbStatus = ClimbState.STOP;
        }

        switch (climbStatus) {
            case STOP:
                climb_talon_1.set(0);
                climb_talon_2.set(0);
                break;
            case CLIMB:
                // Pull holder back to prevent interference with ground
                // robot.holder.retract();

                // Climbing with current threshold
                if (avgCurrent < 50) {
                    climb_talon_1.set(climbRate);
                    climb_talon_2.set(climbRate);
                } else {
                    climbStatus = ClimbState.STOP;
                }
                break;
            case FORCE_CLIMB:
                // robot.xbox.setToggled(climbButton, false);
                climb_talon_1.set(climbRate);
                climb_talon_2.set(climbRate);
                break;
        }
    }

    private void avgCurrent() {
        i = 0;
        current[i] = (pdp.getCurrent(13) + pdp.getCurrent(14)) / 2;
        if (i < 4) {
            i++;
        } else {
            i = 0;
        }
        totalCurrent = 0;
        for (double amps : current) {
            totalCurrent += amps;
        }
        avgCurrent = totalCurrent / 5.0;
    }
}