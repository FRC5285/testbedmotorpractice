package frc.robot;

/*
USAGE EXAMPLE

ledSubsystem.currentPattern = new double[]{1.0, skyBlue, hotPink, white, hotPink, skyBlue}; //first index is speed in Hz
ledSubsystem.ledPatternState = null; //required: resets anim
ledSubssytem.currentPattern - new double[]{1.0, off}; //turns the LED off
 */



public final class ledConstants {

    public static class Constants {
        public static final int led_pin = 0;

        public static final double hotPink = 0.57; //trans flag colours
        public static final double skyBlue = 0.83; //warning before shift change
        public static final double blue = 0.87; 
        public static final double white = 0.93;

        public static final double red = 0.57; //reserved for warnings + urgent shit

        public static final double rainbow_rainbow = -0.99; //rainbow pattern and rainbow colours

        public static final double off = 0.99;

    }
}
