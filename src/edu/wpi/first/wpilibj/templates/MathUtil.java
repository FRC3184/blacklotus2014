/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author Alpha
 */
public class MathUtil {
    public static double cube(double n) {
        return n*n*n;
    }
    public static boolean compareDouble(double d1, double d2, double diff) {
        return Math.abs(d1-d2) < diff;
    }
    public static double removeJitter(double value) {
        if (Math.abs(value) < .05) {
            return 0;
        }
        return value;
    }
}
