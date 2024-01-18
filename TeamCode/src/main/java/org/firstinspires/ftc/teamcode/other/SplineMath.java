package Spline;

public class SplineMath {
    /**
     * adds angles (please use radians )
     * @param angleOne
     * @param angleTwo
     * @return added angles bounded between [-π, π]
     */
    public static double addAngles(double angleOne, double angleTwo){
        double sum = angleOne + angleTwo;
        if(sum > Math.PI){
            sum = sum - 2 * Math.PI;
        }else if(sum < -Math.PI){
            sum = sum + 2 * Math.PI;
        }

        return sum;
    }

    public static double clip(double value, double min, double max){
        if(value < min){
            return min;
        }else if(value > max){
            return max;
        }else{
            return value;
        }

    }

}
