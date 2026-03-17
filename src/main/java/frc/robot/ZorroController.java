package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public class ZorroController extends GenericHID {

    /*//private int zorroPort;
    private int UNKNOWNPORT = -1;
    private static final String ZORROTEST = "Zorro";

    private Joystick zorroController;*/

    public ZorroController(final int zorroPort) {
        super(zorroPort);
        //HAL.report(tResourceType.kResourceType_XboxController, port + 1);

        /*zorroPort = getZorroPort();

        if (zorroPort != UNKNOWNPORT) {
            zorroController = new Joystick(zorroPort);
        }*/
    }

    /*public int getZorroPort() {
        int portNum;
        GenericHID genericController;

        for (portNum = 0; portNum < 6; portNum++) {
            genericController = new GenericHID(portNum);

            if (genericController.getName().equals(ZORROTEST) == true) {
                System.out.println("Zorro Controller Connected");
                return portNum;
            }
        }

        System.out.println("Could not find Zorro Controller");
        return UNKNOWNPORT;
    }*/

    // Zorro joystick axis
    public enum Axis {
        kRightX(0),
        kRightY(1),
        // Spend time reevaluating joysticks
        //kLeftDial(2),
        //kRightDial(3),
        kLeftX(3),
        kLeftY(2),
        kE(4),
        kF(5),
        kB(7),
        kC(6);

        public final int value;

        Axis(int value) {
            this.value = value;
        }

        @Override
        public String toString() {
            var name = this.name().substring(1);
            //If statement probably doesn't need to be there:
            if (name.endsWith("Dial")) {
                return name + "Axis";
            }
            return name;
        }
    }

    // Zorro buttons
    public enum Button {
        kA(1),
        kD(2),
        kG(3),
        kH(4),
        kEUp(5),
        kEDown(4),
        // kFUp(12),
        kFUp(12),
        kFDown(13),
        kBUp(3),
        kBMid(999),
        kBDown(99),
        kCUp(10),
        kCMid(9),
        kCDown(8);

        public final int value;

        Button(int value) {
            this.value = value;
        }

        @Override
        public String toString() {
            var name = this.name().substring(1);
            if (name.endsWith("Up") || name.endsWith("Mid") || name.endsWith("Down")) {
                return name;
            }
            return name + "Button";
        }
    }

    /*public static int ZorroLeftXAxis = 0;
    public static int ZorroLeftYAxis = 1;

    public static int ZorroLeftDial = 2;
    public static int ZorroRightDial = 3;

    public static int ZorroRightXAxis = 4;
    public static int ZorroRightYAxis = 5;

    public static int ZorroA = 6;
    public static int ZorroD = 13;
    public static int ZorroG = 7;
    public static int ZorroH = 14;

    public static int ZorroEUp = 5;
    public static int ZorroEDown = 4;

    public static int ZorroFUp = 12;
    public static int ZorroFDown = 11;

    public static int ZorroBUp = 3;
    public static int ZorroBMid = 2;
    public static int ZorroBDown = 1;

    public static int ZorroCUp = 10;
    public static int ZorroCMid = 9;
    public static int ZorroCDown = 8;*/

    // Zorro Controls

    public double getLeftX() {
        return getRawAxis(Axis.kLeftX.value);
    }

    public double getRightX() {
        return getRawAxis(Axis.kRightX.value);
    }

    public double getLeftY() {
        return getRawAxis(Axis.kLeftY.value);
    }

    public double getRightY() {
        return getRawAxis(Axis.kRightY.value);
    }

    /*public double getLeftDial() {
        return getRawAxis(Axis.kLeftDial.value);
    }

    public double getRightDial() {
        return getRawAxis(Axis.kRightDial.value);
    }*/

    // A Button
    public boolean getAButton() {
        return getRawButton(Button.kA.value);
    }

    public boolean getAButtonPressed() {
        return getRawButtonPressed(Button.kA.value);
    }

    public boolean getAButtonReleased() {
        return getRawButtonReleased(Button.kA.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent a(EventLoop loop) {
        return new BooleanEvent(loop, this::getAButton);
    }

    // D Button
    public boolean getDButton() {
        return getRawButton(Button.kD.value);
    }

    public boolean getDButtonPressed() {
        return getRawButtonPressed(Button.kD.value);
    }

    public boolean getDButtonReleased() {
        return getRawButtonReleased(Button.kD.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent d(EventLoop loop) {
        return new BooleanEvent(loop, this::getDButton);
    }

    // G Button
    public boolean getGButton() {
        return getRawButton(Button.kG.value);
    }

    public boolean getGButtonPressed() {
        return getRawButtonPressed(Button.kG.value);
    }

    public boolean getGButtonReleased() {
        return getRawButtonReleased(Button.kG.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent g(EventLoop loop) {
        return new BooleanEvent(loop, this::getGButton);
    }

    // H Button
    public boolean getHButton() {
        return getRawButton(Button.kH.value);
    }

    public boolean getHButtonPressed() {
        return getRawButtonPressed(Button.kH.value);
    }

    public boolean getHButtonReleased() {
        return getRawButtonReleased(Button.kH.value);
    }

    @SuppressWarnings("MethodName")
    public BooleanEvent h(EventLoop loop) {
        return new BooleanEvent(loop, this::getHButton);
    }

    // E switch/axis
    // Up = 1, Down = -1
    public boolean getETwoPosSwitch() {
        if (getRawAxis(Axis.kE.value) == -1) {
            return false;
        }
        return true;
    }

    // F Switch/axis
    // Up = true (1), Down = false (-1)
    public boolean getFTwoPosSwitch() {
        if (getRawAxis(Axis.kF.value) == -1) {
            return false;
        }
        return true;
    }

    // B tri-state switch/axis
    // Up = (1), Down = (-1), Center = (0)
    public double getBThreePosSwitch() {
        return getRawAxis(Axis.kB.value);
    }

    // C tri-state switch/axis
    // Up = (1), Down = (-1), Center = (0)
    public double getCThreePosSwitch() {
        return getRawAxis(Axis.kC.value);
    }
}
