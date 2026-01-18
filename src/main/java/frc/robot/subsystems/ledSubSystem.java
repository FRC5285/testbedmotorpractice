package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.ledConstants.Constants;


public class ledSubSystem extends SubsystemBase {
    private final Spark LED = new Spark(Constants.led_pin);
    public long[] ledPatternState = null;
    public double[] currentPattern = null;
    private double currentValue = 0.0;
    


    public void ledPattern(double[] arr) {
    if (arr == null || arr.length < 2) return;

    long now = System.currentTimeMillis();

    if (ledPatternState == null) {
        ledPatternState = new long[]{now, 1, (long)(1000 / arr[0])};
        currentPattern = arr;
        currentValue = arr[1];
        return;
    }

    if (now - ledPatternState[0] >= ledPatternState[2]) {
        int index = (int) ledPatternState[1] + 1;
        if (index >= arr.length) index = 1;

        currentValue = arr[index];
        ledPatternState[0] = now;
        ledPatternState[1] = index;
    }
}

    public void setPattern(double[] pattern) {
        ledPatternState = null;
        currentPattern = pattern;
    }

    
    public Command stopMotor() {
        return runOnce(() -> LED.stopMotor());
    }
    @Override
    public void periodic() {
    
    if (currentPattern != null) {
        ledPattern(currentPattern);
        LED.set(currentValue);
    }

}
}


