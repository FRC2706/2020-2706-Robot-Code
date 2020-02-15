package frc.robot.config;

public enum XboxValue {
    // Axis and triggers
    XBOX_LEFT_STICK_X(0, XboxInputType.Axis),
    XBOX_LEFT_STICK_Y(1, XboxInputType.Axis),
    XBOX_BACK_LEFT_TRIGGER(2, XboxInputType.Axis),
    XBOX_BACK_RIGHT_TRIGGER(3, XboxInputType.Axis),
    XBOX_RIGHT_STICK_X(4,  XboxInputType.Axis),
    XBOX_RIGHT_STICK_Y(5, XboxInputType.Axis),

    // Buttons
    XBOX_A_BUTTON(1, XboxInputType.Button),
    XBOX_B_BUTTON(2, XboxInputType.Button),
    XBOX_X_BUTTON(3, XboxInputType.Button),
    XBOX_Y_BUTTON(4, XboxInputType.Button),
    XBOX_LB_BUTTON(5, XboxInputType.Button),
    XBOX_RB_BUTTON(6, XboxInputType.Button),
    XBOX_SELECT_BUTTON(7, XboxInputType.Button),
    XBOX_START_BUTTON(8,  XboxInputType.Button),
    XBOX_LEFT_AXIS_BUTTON(9, XboxInputType.Button),
    XBOX_RIGHT_AXIS_BUTTON(10, XboxInputType.Button),

    // POV (The D-PAD on the XBOX Controller)
    XBOX_POV_UP(0,  XboxInputType.POV),
    XBOX_POV_UP_RIGHT(45,  XboxInputType.POV),
    XBOX_POV_RIGHT(90, XboxInputType.POV),
    XBOX_POV_DOWN_RIGHT(135, XboxInputType.POV),
    XBOX_POV_DOWN(180, XboxInputType.POV),
    XBOX_POV_DOWN_LEFT(225, XboxInputType.POV),
    XBOX_POV_LEFT(270, XboxInputType.POV),
    XBOX_POV_UP_LEFT(315, XboxInputType.POV);


    XboxValue(int port, XboxInputType type) {
        this.port = port;
        this.type = type;
    }

    private final int port;

    private final XboxInputType type;

    public int getPort() {
        return port;
    }

    public XboxInputType getType() {
        return type;
    }

    /**
     * The type of input
     */
    public enum XboxInputType {
        /**
         * A raw axis or analog stick
         */
        Axis,

        /**
         * A button on the joystick
         */
        Button,

        /**
         * A POV or D-pad on the joystick
         */
        POV
    }

}