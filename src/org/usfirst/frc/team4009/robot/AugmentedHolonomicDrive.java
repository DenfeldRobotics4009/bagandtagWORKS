package org.usfirst.frc.team4009.robot;

import  edu.wpi.first.wpilibj.MotorSafety;
import  edu.wpi.first.wpilibj.MotorSafetyHelper;
import  edu.wpi.first.wpilibj.SpeedController;

public class AugmentedHolonomicDrive implements MotorSafety {
    protected MotorSafetyHelper m_motorSafetyHelper;

    protected SpeedController
        m_frontLeftMotor,
        m_leftMotor,
        m_rearLeftMotor,
        m_frontRightMotor,
        m_rightMotor,
        m_rearRightMotor;

    public AugmentedHolonomicDrive(
            SpeedController frontLeftMotor,
            SpeedController leftMotor,
            SpeedController rearLeftMotor,
            SpeedController frontRightMotor,
            SpeedController rightMotor,
            SpeedController rearRightMotor)
    {
        boolean nullCheck = (frontLeftMotor == null);
        nullCheck |= (leftMotor == null);
        nullCheck |= (rearLeftMotor == null);
        nullCheck |= (frontRightMotor == null);
        nullCheck |= (rightMotor == null);
        nullCheck |= (rearRightMotor == null);
        if(nullCheck){
            m_leftMotor = m_rightMotor = m_frontLeftMotor = m_rearLeftMotor = m_frontRightMotor = m_rearRightMotor = null;
            throw new NullPointerException("Null motor provided");
        }

        m_frontLeftMotor = frontLeftMotor;
        m_leftMotor = leftMotor;
        m_rearLeftMotor = rearLeftMotor;
        m_frontRightMotor = frontRightMotor;
        m_rightMotor = rightMotor;
        m_rearRightMotor = rearRightMotor;

        m_frontRightMotor.setInverted(true);
        m_rightMotor.setInverted(true);
        m_rearRightMotor.setInverted(true);

        setupMotorSafety();
        stopMotor();
    }

    protected static double bound(double var){
        var = (var > 1.0) ? 1.0 : var;
        var = (var < -1.0) ? -1.0 : var;
        return var;
    }

    public void drive(double x, double y, double rotation){
        drive(x,y,rotation,true);
    }

    public void drive(double x, double y, double rotation, boolean square){
        if(square){
            x = (x>0.0) ? x*x : -x*x;
            y = (y>0.0) ? y*y : -y*y;
            rotation = (rotation>0.0) ? rotation*rotation : -rotation*rotation;
        }

        double left, right;

        rotation = -rotation;//dirty hack start
        if (y > 0.0) {
            if (rotation > 0.0) {
                left = y - rotation;
                right = Math.max(y, rotation);
            } else {
                left = Math.max(y, -rotation);
                right = y + rotation;
            }
        } else {
            if (rotation > 0.0) {
                left = -Math.max(-y, rotation);
                right = y + rotation;
            } else {
                left = y - rotation;
                right = -Math.max(-y, -rotation);
            }
        }
        rotation = -rotation;//dirty hack end

        double augmentation_x = 0; //skidsteer can't strafe
        double augmentation_y = left + right;

        double stick_magnitude = Math.sqrt(x*x + y*y);

        double stick_dot = dot(x,y,x,y);
        double rejection_x = 0;
        double rejection_y = 0;
        if(stick_dot == 0.0){
            rejection_x = augmentation_x - (dot(augmentation_x,augmentation_y,x,y)/stick_dot)*x;
            rejection_y = augmentation_y - (dot(augmentation_x,augmentation_y,x,y)/stick_dot)*y;
        }

        double corrected_x = x - rejection_x;
        double corrected_y = y - rejection_y;

        double speeds[] = new double[6];
        speeds[0] = right;
        speeds[1] = left;
        speeds[2] = corrected_x + corrected_y + rotation;
        speeds[3] = -corrected_x + corrected_y - rotation;
        speeds[4] = -corrected_x + corrected_y + rotation;
        speeds[5] = corrected_x + corrected_y - rotation;

        normalize(speeds, 6);

        m_rightMotor.set(speeds[0]);
        m_leftMotor.set(speeds[1]);

        m_frontLeftMotor.set(speeds[2]);
        m_frontRightMotor.set(speeds[3]);
        m_rearLeftMotor.set(speeds[4]);
        m_rearRightMotor.set(speeds[5]);

        if (m_motorSafetyHelper != null) m_motorSafetyHelper.feed();
    }

    private double dot(double x1, double y1, double x2, double y2){
        return x1*x2 + y1*y2;
    }

    static void normalize(double speeds[], int size) {
        double max = Math.abs(speeds[0]);
        int i;
        for (i = 1; i < size; i++) {
            double temp = Math.abs(speeds[i]);
            max = (max > temp) ? max : temp;
        }
        if (max > 1.0) {
            for (i = 0; i < size; i++) {
                speeds[i] = speeds[i] / max;
            }
        }
    }

    public void setExpiration(double timeout) {
        m_motorSafetyHelper.setExpiration(timeout);
    }

    public double getExpiration() {
        return m_motorSafetyHelper.getExpiration();
    }

    public boolean isAlive() {
        return m_motorSafetyHelper.isAlive();
    }

    public boolean isSafetyEnabled() {
        return m_motorSafetyHelper.isSafetyEnabled();
    }

    public void setSafetyEnabled(boolean enabled) {
        m_motorSafetyHelper.setSafetyEnabled(enabled);
    }

    public String getDescription(){
        return "Augmented Holonomic Drive";
    }

    public void stopMotor() {
        m_frontLeftMotor.set(0);
        m_leftMotor.set(0);
        m_rearLeftMotor.set(0);
        m_frontRightMotor.set(0);
        m_rightMotor.set(0);
        m_rearRightMotor.set(0);
    }

    private void setupMotorSafety() {
        m_motorSafetyHelper = new MotorSafetyHelper(this);
        m_motorSafetyHelper.setExpiration(DEFAULT_SAFETY_EXPIRATION);
        m_motorSafetyHelper.setSafetyEnabled(true);
    }

}
