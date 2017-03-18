package org.usfirst.frc.team3322;

/**
 * Created by 3322 - Programming 3 on 3/18/2017.
 */


        import edu.wpi.first.wpilibj.GenericHID;
        import edu.wpi.first.wpilibj.Joystick;


public class OI {
    static Joystick joystick;

    // Assign vague integers to variables
    public static final int
            // Buttons
            ABUTTON = 1,
            BBUTTON = 2,
            XBUTTON = 3,
            YBUTTON = 4,
            LBUMPER = 5,
            RBUMPER = 6,
            BACK = 7,
            START = 8,
            LSTICK = 9,
            RSTICK = 10,
            DPADVERT = 7, // not reliable
            DPADHORIZ = 6,

    // Stick axes
    L_YAXIS = 1,
            L_XAXIS = 2,
            R_YAXIS = 3,
            R_XAXIS = 4;

    int buttonState, toggleState;

    public OI() {
        joystick = new Joystick(0);
    }

    // Track button states for pressedOnce
    private void setButtonDown(int button) {
        buttonState |= (1 << button);
    }
    private void setButtonUp(int button) {
        buttonState &= ~(1 << button);
    }
    private boolean isDown(int button) {
        return 0 != (buttonState & (1 << button));
    }

    void toggleButtonState(int button) {
        toggleState ^= (1 << button);
    }

    public boolean heldDown(int button) {
        return joystick.getRawButton(button);
    }
    public boolean pressedOnce(int button) {
        // Returns true only once, and will not return true again until the button is released and pressed again.
        if (heldDown(button)) {
            if (!isDown(button)) {
                setButtonDown(button);
                return true;
            } else {
                return false;
            }
        } else {
            setButtonUp(button);
            return false;
        }
    }

    public double getAxis(int axis) {
        return joystick.getRawAxis(axis);
    }
    public double getFineAxis(int axis, double pow) {
        return Math.abs(Math.pow(joystick.getRawAxis(axis), pow - 1)) * joystick.getRawAxis(axis);
    }

    public boolean isToggled(int button) {
        if (pressedOnce(button)) {
            toggleButtonState(button);
        }
        return 0 != (toggleState & (1 << button));
    }
   /* public void setVibrate(double left, double right) {
        joystick.setRumble(GenericHID.RumbleType.kLeftRumble, left);
        joystick.setRumble(GenericHID.RumbleType.kRightRumble, right);
    }*/
}
