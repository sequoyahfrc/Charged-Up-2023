package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;

public final class Controls {

    public static final Controls INSTANCE = new Controls();
    public final PS4Controller driver1 = new PS4Controller(0);
    public final Joystick driver2 = new Joystick(1);

    public static double getDriver1Forward() {
        return INSTANCE.driver1.getLeftY();
    }

    public static double getDriver1Left() {
        return -INSTANCE.driver1.getLeftX();
    }

    public static double getDriver1TurnCCW() {
        return -INSTANCE.driver1.getRightX();
    }

    public static boolean getDriver1SlowMode() {
        return false;
    }
    
    public static double getSpeedBoost() {
        double x = -INSTANCE.driver2.getRawAxis(3);
        x += 1;
        x /= 2;
        return x;
    }

    public static boolean getDriver1BrickMode() {
        return INSTANCE.driver1.getL2Axis() > 0.5;
    }

    public static boolean getDriver1Cancel() {
        return INSTANCE.driver1.getCircleButtonPressed();
    }

    public static boolean getDriver2Cancel() {
        return INSTANCE.driver2.getRawButtonPressed(2);
    }

    public static boolean getDriver2ManualElevatorUp() {
        int pov = INSTANCE.driver2.getPOV();
        return pov == 0 || pov == 45 || pov == 315;
    }

    public static boolean getDriver2ManualElevatorDown() {
        int pov = INSTANCE.driver2.getPOV();
        return pov == 180 || pov == 135 || pov == 225;
    }

    public static boolean getDriver2ManualExtenderExtend() {
        return INSTANCE.driver2.getRawAxis(1) < -0.25;
    }

    public static boolean getDriver2ManualExtenderRetract() {
        return INSTANCE.driver2.getRawAxis(1) > 0.25;
    }

    public static boolean getDriver2ManualClawUp() {
        return INSTANCE.driver2.getRawButton(5);
    }

    public static boolean getDriver2ManualClawDown() {
        return INSTANCE.driver2.getRawButton(3);
    }

    public static boolean getDriver2Shoot() {
        return INSTANCE.driver2.getRawButtonPressed(1);
    }

    public static boolean getDriver2Intake() {
        return INSTANCE.driver2.getRawButton(6);
    }

    private Controls() {}
}
