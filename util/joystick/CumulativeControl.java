package org.wheelerschool.robotics.robotlib.util.joystick;

public class CumulativeControl {
    private double position;

    private double cps;

    private double lastTime = -1;


    /**
     * Constructor.
     *
     * @param cps: Change per second
     */
    public CumulativeControl(double cps) {
        this.cps = cps;

    }

    /**
     * Set change per second.
     * @param cps: desired rate.
     */
    public void setCps(double cps) {
        this.cps = cps;
    }

    public double getPosition() {
        return position;
    }

    /**
     * Return movement
     * @param time: current time
     * @return double: new movement
     */
    public double getMovementTick(boolean enable, double time) {
        double prevTime = lastTime;  // Save previous time

        if (!enable) {
            // Disabled, so flag as such
            lastTime = -1;
        } else {
            // Update the last measured time, to current
            lastTime = time;
        }

        if (prevTime == -1) {
            // Return no movement if was disabled
            return 0;
        }

        // Else: return change (time elapsed * rate):
        return (time - prevTime) * cps;
    }

    public void positionTick(boolean enable, double time, int direction) {
        position += getMovementTick(enable, time)*direction;
    }

    public void positionTick(boolean enable, double time) {
        positionTick(enable, time, 1);
    }
}
