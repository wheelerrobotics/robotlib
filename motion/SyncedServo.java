package org.wheelerschool.robotics.robotlib.motion;

import com.qualcomm.robotcore.hardware.Servo;

public class SyncedServo {
    /**
     * Easily handle multiple servos together;
     */
    Servo[] servos;

    public SyncedServo(Servo[] servos) {
        this.servos = servos;
    }

    /**
     * Set position of all servos.
     */
    public void setPosition(double position) {
        for (Servo s : servos) {
            s.setPosition(position);
        }
    }

    /**
     * Return average position between servos.
     */
    public double getPosition() {


        double cumSum = 0;
        for (Servo s : servos) {
            cumSum += s.getPosition();
        }
        return cumSum / servos.length;
    }
}
