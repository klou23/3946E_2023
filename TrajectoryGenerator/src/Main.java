/**
 * File: Main.java
 * Author: Kevin Lou
 * Created On: Dec 23 2022
 * Last Updated: Dec 27 2022
 *
 * Copyright (c) 2022 3946E
 *
 * Summary of file:
 *  - Contains the entire algorithm to generate a robot trajectory that is
 *    compatible with the RAMSETE controller.
 */

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.StringTokenizer;

public class Main {

    final static String pathName = "DriveTest";

    final static double decompositionInterval = 0.005; //seconds

    static List<Double> x = new ArrayList<>();
    static List<Double> y = new ArrayList<>();
    static List<Double> theta = new ArrayList<>();
    static List<Double> v = new ArrayList<>();
    static List<Double> omega = new ArrayList<>();

    static BufferedReader br;

    public static void main(String[] args) throws IOException {
        //reads input
        br = new BufferedReader(new FileReader("Input/Path" + pathName + ".txt"));
        String line;
        while((line = br.readLine()) != null){
            if(line.equals("DRIVE_PATH")) drivePath();
            if(line.equals("HOLD")) hold();
            if(line.equals("TURN_IN_PLACE")) turnInPlace();
            if(line.equals("DRIVE_REV")) driveRev();
        }

        //output
        StringBuilder sb = new StringBuilder();
        sb.append(x.size() + "\n");
        for(int i = 0; i < x.size(); i++){
            sb.append(String.format("%.5f    %.5f    %.5f    %.5f    %.5f\n",
                                    x.get(i), y.get(i), theta.get(i), v.get(i), omega.get(i)));
        }
        PrintWriter pw = new PrintWriter("Output/Path" + pathName + ".txt");
        pw.print(sb);
        pw.close();
    }

    /**
     * Computes the trajectory for a HOLD command
     * @throws IOException
     */
    public static void hold() throws IOException {
        //read input
        StringTokenizer st = new StringTokenizer(br.readLine());
        double time = Double.parseDouble(st.nextToken());
        time /= 1000.0;

        double xPos = (x.isEmpty() ? 0 : x.get(x.size()-1));
        double yPos = (y.isEmpty() ? 0 : y.get(y.size()-1));
        double thetaPos = (theta.isEmpty() ? 0 : theta.get(theta.size()-1));

        //compute trajectory
        for(double i = 0; i < time; i += decompositionInterval){
            x.add(xPos);
            y.add(yPos);
            theta.add(thetaPos);
            v.add(0.0);
            omega.add(0.0);
        }
    }

    /**
     * Computes the trajectory for a TURN_IN_PLACE command
     * @throws IOException
     */
    public static void turnInPlace() throws IOException{
        //read input
        StringTokenizer st = new StringTokenizer(br.readLine());
        double maxOmega = Double.parseDouble(st.nextToken());
        double maxAlpha = Double.parseDouble(st.nextToken());

        st = new StringTokenizer(br.readLine());
        double theta_f = Double.parseDouble(st.nextToken());
        double theta_0 = (theta.isEmpty() ? 0 : theta.get(theta.size()-1));

        //compute
        double thetaAccel = (maxOmega*maxOmega)/(2*maxAlpha);

        double xPos = (x.isEmpty() ? 0 : x.get(x.size()-1));
        double yPos = (y.isEmpty() ? 0 : y.get(y.size()-1));

        if(2*thetaAccel > Math.abs(theta_f - theta_0)){
            double thetaHalf = Math.abs(theta_f - theta_0)/2;
            double t = Math.sqrt(2*thetaHalf/maxAlpha);

            double curAlpha = (theta_0 < theta_f ? maxAlpha : -maxAlpha);
            double curTheta = theta_0;
            double curOmega = 0;

            //accelerate
            for(double i = 0; i < t; i += decompositionInterval){
                x.add(xPos);
                y.add(yPos);
                v.add(0.0);
                theta.add(curTheta);
                omega.add(curOmega);

                curTheta += curOmega*decompositionInterval + 0.5*curAlpha*Math.pow(decompositionInterval, 2);
                curOmega += curAlpha*decompositionInterval;
            }

            //decelerate
            curAlpha *= -1;
            for(double i = 0; i < t; i += decompositionInterval){
                x.add(xPos);
                y.add(yPos);
                v.add(0.0);
                theta.add(curTheta);
                omega.add(curOmega);

                curTheta += curOmega*decompositionInterval + 0.5*curAlpha*Math.pow(decompositionInterval, 2);
                curOmega += curAlpha*decompositionInterval;
            }
        }else{
            double t = Math.sqrt(2*thetaAccel/maxAlpha);

            double curAlpha = (theta_0 < theta_f ? maxAlpha : -maxAlpha);
            double curTheta = theta_0;
            double curOmega = 0;

            //accelerate
            for(double i = 0; i < t; i += decompositionInterval){
                x.add(xPos);
                y.add(yPos);
                v.add(0.0);
                theta.add(curTheta);
                omega.add(curOmega);

                curTheta += curOmega*decompositionInterval + 0.5*curAlpha*Math.pow(decompositionInterval, 2);
                curOmega += curAlpha*decompositionInterval;
                if(curOmega > maxOmega) curOmega = maxOmega;
                if(curOmega < -maxOmega) curOmega = -maxOmega;
            }

            //constant vel
            double constantT = (Math.abs(theta_f - theta_0)-2*Math.abs(curTheta-theta_0))/(curOmega);
            for(double i = 0; i < constantT; i+= decompositionInterval){
                x.add(xPos);
                y.add(yPos);
                v.add(0.0);
                theta.add(curTheta);
                omega.add(curOmega);

                curTheta += curOmega*decompositionInterval;
            }

            //decelerate
            curAlpha *= -1;
            do{
                x.add(xPos);
                y.add(yPos);
                v.add(0.0);
                theta.add(curTheta);
                omega.add(curOmega);

                curTheta += curOmega*decompositionInterval + 0.5*curAlpha*Math.pow(decompositionInterval, 2);
                curOmega += curAlpha*decompositionInterval;
            } while(curOmega > 0);

            //final point
            if(omega.get(omega.size()-1) > 0.00001){
                x.add(xPos);
                y.add(yPos);
                v.add(0.0);
                theta.add(curTheta);
                omega.add(0.0);
            }
        }
    }

    /**
     * Computes the trajectory for a DRIVE_PATH command
     * @throws IOException
     */
    public static void drivePath() throws IOException {
        //read input
        StringTokenizer st = new StringTokenizer(br.readLine());
        double maxVel = Double.parseDouble(st.nextToken());
        double maxAccel = Double.parseDouble(st.nextToken());
        double turnSpeedConstant = Double.parseDouble(st.nextToken());
        int waypointCnt = Integer.parseInt(st.nextToken());

        st = new StringTokenizer(br.readLine());
        BezierStartPoint startPoint = new BezierStartPoint(Double.parseDouble(st.nextToken()),
                                                           Double.parseDouble(st.nextToken()),
                                                           Double.parseDouble(st.nextToken()),
                                                           Double.parseDouble(st.nextToken()));

        List<BezierWaypoint> waypoints = new ArrayList<>();
        for(int i = 0; i < waypointCnt; i++){
            st = new StringTokenizer(br.readLine());
            BezierWaypoint waypoint = new BezierWaypoint(Double.parseDouble(st.nextToken()),
                                                         Double.parseDouble(st.nextToken()),
                                                         Double.parseDouble(st.nextToken()),
                                                         Double.parseDouble(st.nextToken()),
                                                         Double.parseDouble(st.nextToken()));
            waypoints.add(waypoint);
        }

        st = new StringTokenizer(br.readLine());
        BezierEndPoint endPoint = new BezierEndPoint(Double.parseDouble(st.nextToken()),
                                                     Double.parseDouble(st.nextToken()),
                                                     Double.parseDouble(st.nextToken()),
                                                     Double.parseDouble(st.nextToken()));

        st = new StringTokenizer(br.readLine());
        double endVel = Integer.parseInt(st.nextToken());

        //compute path
        Point[] points = bezier(startPoint, waypoints, endPoint);
        double[] dist = calculateDistance(points);
        double[] curvature = calculateCurvature(points);
        double[] vels = calculateVels(points, curvature, dist, maxVel, maxAccel, turnSpeedConstant, (v.isEmpty() ? 0 : v.get(v.size()-1)), endVel);

        int startIndex = v.size();
        selectPoints(points, vels, dist);
        calculateTheta(startIndex);
        calculateOmega(startIndex);
    }

    public static void driveRev() throws IOException {
        //read input
        StringTokenizer st = new StringTokenizer(br.readLine());
        double maxVel = Double.parseDouble(st.nextToken());
        double maxAccel = Double.parseDouble(st.nextToken());

        st = new StringTokenizer(br.readLine());
        double dist = Double.parseDouble(st.nextToken());

        double x0 = (x.isEmpty() ? 0 : x.get(x.size()-1));
        double y0 = (y.isEmpty() ? 0 : y.get(y.size()-1));
        double theta0 = (theta.isEmpty() ? 0 : theta.get(theta.size()-1));

        List<Point> points = new ArrayList<>();
        for(double i = 0; i <= 1; i += 0.0001){
            points.add(new Point(x0, y0));
            x0 -= Math.cos(theta0)*dist*0.0001;
            y0 -= Math.sin(theta0)*dist*0.0001;
        }

        Point[] path = new Point[points.size()];
        for(int i = 0; i < points.size(); i++){
            path[i] = points.get(i);
        }

        double[] distArr = calculateDistance(path);

        int pathLen = path.length;
        double[] vels = new double[pathLen];

        for(int i = 0; i < pathLen; i++){
            vels[i] = 1<<30;
        }

        vels[pathLen-1] = 0;
        for(int i = pathLen-2; i >= 0; i--){
            double distance = distArr[i+1]-distArr[i];
            vels[i] = Math.min(vels[i], Math.sqrt(vels[i+1]*vels[i+1] + 2*maxAccel*distance));
        }

        vels[0] = 0;
        for(int i = 1; i < pathLen; i++){
            double distance = distArr[i]-distArr[i-1];
            vels[i] = Math.min(vels[i], Math.sqrt(vels[i-1]*vels[i-1] + 2*maxAccel*distance));
        }

        for(int i = 0; i < pathLen; i++){
            vels[i] *= -1;
        }

        x.add(path[0].x);
        y.add(path[0].y);
        v.add(vels[0]);
        theta.add(theta0);
        omega.add(0.0);

        double timeElapsed = 0;
        for(int i = 1; i < path.length; i++){
            double distance = distArr[i]-distArr[i-1];
            double time = Math.abs(distance/((vels[i-1]+vels[i])/2));
            timeElapsed += time;
            if(timeElapsed > decompositionInterval){
                x.add(path[i].x);
                y.add(path[i].y);
                v.add(vels[i]);
                theta.add(theta0);
                omega.add(0.0);
                timeElapsed = 0;
            }
        }

        if(x.get(x.size()-1) != path[path.length-1].x ||
                y.get(y.size()-1) != path[path.length-1].y ||
                v.get(v.size()-1) != vels[vels.length-1]){
            x.add(path[path.length-1].x);
            y.add(path[path.length-1].y);
            v.add(vels[vels.length-1]);
            omega.add(0.0);
            theta.add(theta0);
        }
    }

    /**
     * Smooths the path points using a quadratic bezier spline algorithm
     * @param startPoint start point for the path
     * @param waypoints list of waypoints in the path
     * @param endPoint end point for the path
     * @return list of points. 10,000 points for each segment of the path
     */
    public static Point[] bezier(BezierStartPoint startPoint, List<BezierWaypoint> waypoints,
                                     BezierEndPoint endPoint){
        List<Point> sol = new ArrayList<>();
        double p0x, p0y, p1x, p1y, p2x, p2y, p3x, p3y;
        if(waypoints.size() == 0){
            //segment between start and end
            p0x = startPoint.getStartPoint().x;
            p0y = startPoint.getStartPoint().y;
            p1x = startPoint.getControlPoint().x;
            p1y = startPoint.getControlPoint().y;
            p2x = endPoint.getControlPoint().x;
            p2y = endPoint.getControlPoint().y;
            p3x = endPoint.getEndPoint().x;
            p3y = endPoint.getEndPoint().y;
            for(double t = 0; t <= 1; t += 0.0001){
                double bx = Math.pow(1-t, 3)*p0x + 3*Math.pow(1-t,2)*t*p1x + 3*(1-t)*t*t*p2x + Math.pow(t,3)*p3x;
                double by = Math.pow(1-t, 3)*p0y + 3*Math.pow(1-t,2)*t*p1y + 3*(1-t)*t*t*p2y + Math.pow(t,3)*p3y;
                sol.add(new Point(bx, by));
            }
        }else{
            //segment between start and first waypoint
            p0x = startPoint.getStartPoint().x;
            p0y = startPoint.getStartPoint().y;
            p1x = startPoint.getControlPoint().x;
            p1y = startPoint.getControlPoint().y;
            p2x = waypoints.get(0).getBeforeControlPoint().x;
            p2y = waypoints.get(0).getBeforeControlPoint().y;
            p3x = waypoints.get(0).getPoint().x;
            p3y = waypoints.get(0).getPoint().y;
            for(double t = 0; t <= 1; t += 0.0001){
                double bx = Math.pow(1-t, 3)*p0x + 3*Math.pow(1-t,2)*t*p1x + 3*(1-t)*t*t*p2x + Math.pow(t,3)*p3x;
                double by = Math.pow(1-t, 3)*p0y + 3*Math.pow(1-t,2)*t*p1y + 3*(1-t)*t*t*p2y + Math.pow(t,3)*p3y;
                sol.add(new Point(bx, by));
            }

            //segments between waypoints
            for(int i = 0; i < waypoints.size()-1; i++){
                p0x = waypoints.get(i).getPoint().x;
                p0y = waypoints.get(i).getPoint().y;
                p1x = waypoints.get(i).getAfterControlPoint().x;
                p1y = waypoints.get(i).getAfterControlPoint().y;
                p2x = waypoints.get(i+1).getBeforeControlPoint().x;
                p2y = waypoints.get(i+1).getBeforeControlPoint().y;
                p3x = waypoints.get(i+1).getPoint().x;
                p3y = waypoints.get(i+1).getPoint().y;
                for(double t = 0; t <= 1; t += 0.0001){
                    double bx = Math.pow(1-t, 3)*p0x + 3*Math.pow(1-t,2)*t*p1x + 3*(1-t)*t*t*p2x + Math.pow(t,3)*p3x;
                    double by = Math.pow(1-t, 3)*p0y + 3*Math.pow(1-t,2)*t*p1y + 3*(1-t)*t*t*p2y + Math.pow(t,3)*p3y;
                    sol.add(new Point(bx, by));
                }
            }

            //segment between last waypoint and end
            p0x = waypoints.get(waypoints.size()-1).getPoint().x;
            p0y = waypoints.get(waypoints.size()-1).getPoint().y;
            p1x = waypoints.get(waypoints.size()-1).getAfterControlPoint().x;
            p1y = waypoints.get(waypoints.size()-1).getAfterControlPoint().y;
            p2x = endPoint.getControlPoint().x;
            p2y = endPoint.getControlPoint().y;
            p3x = endPoint.getEndPoint().x;
            p3y = endPoint.getEndPoint().y;
            for(double t = 0; t <= 1; t += 0.0001){
                double bx = Math.pow(1-t, 3)*p0x + 3*Math.pow(1-t,2)*t*p1x + 3*(1-t)*t*t*p2x + Math.pow(t,3)*p3x;
                double by = Math.pow(1-t, 3)*p0y + 3*Math.pow(1-t,2)*t*p1y + 3*(1-t)*t*t*p2y + Math.pow(t,3)*p3y;
                sol.add(new Point(bx, by));
            }
        }
        Point[] ret = new Point[sol.size()];
        for(int i = 0; i < sol.size(); i++) ret[i] = sol.get(i);
        return ret;
    }

    /**
     * Calculates the distance from the start of the path to each point using a dynamic programming algorithm
     * @param points path points
     * @return array of distances where the i-th value is the distance from the start of the path to point i
     */
    public static double[] calculateDistance(Point[] points){
        int n = points.length;
        double[] sol = new double[n];
        sol[0] = 0;
        for(int i = 1; i < n; i++){
            sol[i] = sol[i-1] + points[i-1].distTo(points[i]);
        }
        return sol;
    }

    /**
     * Calculates the curvature of each point along the path. The curvature of the endpoints are calculated as zero
     * @param points path points
     * @return array of curvatures where the i-th value is the curvature of the path at point i
     */
    public static double[] calculateCurvature(Point[] points){
        int n = points.length;
        double[] sol = new double[n];
        for(int i = 1; i < n-1; i++){
            sol[i] = points[i].findCurvature(points[i-1], points[i+1]);
        }
        return sol;
    }

    /**
     *
     * @param path array of path points
     * @param curvature array of curvatures at different points along the path
     * @param distance array of distances from the start of the path to each point
     * @param maxVel maximum robot velocity
     * @param maxAccel maximum robot acceleration
     * @param turnSpeedConstant constant limiting velocity around turns
     * @param startVel velocity at the start of the path
     * @param endVel velocity at the end of the path
     * @return array of target velocities where the i-th value is the target velocity for point i
     */
    public static double[] calculateVels(Point[] path, double[] curvature, double[] distance, double maxVel,
                                         double maxAccel, double turnSpeedConstant, double startVel, double endVel){
        int n = path.length;
        double[] sol = new double[n];

        //limit velocity using curvature
        for(int i = 0; i < n; i++){
            if(curvature[i] == 0) sol[i] = maxVel;
            else sol[i] = Math.min(maxVel, maxVel * turnSpeedConstant/curvature[i]);
        }

        //limit velocity using braking
        sol[n-1] = endVel;
        for(int i = n-2; i >= 0; i--){
            double dist = distance[i+1]-distance[i];
            sol[i] = Math.min(sol[i], Math.sqrt(sol[i+1]*sol[i+1] + 2*maxAccel*dist));
        }

        //limit velocity using acceleration
        sol[0] = startVel;
        for(int i = 1; i < n; i++){
            double dist = distance[i]-distance[i-1];
            sol[i] = Math.min(sol[i], Math.sqrt(sol[i-1]*sol[i-1] + 2*maxAccel*dist));
        }

        return sol;
    }

    /**
     * Selects points from the path such that the time between the points is decompositionInterval
     * @param path array of path points
     * @param vels array of velocities
     * @param dist array of distances from the start of the path to each point
     */
    public static void selectPoints(Point[] path, double[] vels, double[] dist){
        x.add(path[0].x);
        y.add(path[0].y);
        v.add(vels[0]);
        double timeElapsed = 0;
        for(int i = 1; i < path.length; i++){
            double distance = dist[i]-dist[i-1];
            double time = distance/((vels[i-1]+vels[i])/2);
            timeElapsed += time;
            if(timeElapsed > decompositionInterval){
                x.add(path[i].x);
                y.add(path[i].y);
                v.add(vels[i]);
                timeElapsed = 0;
            }
        }

        if(x.get(x.size()-1) != path[path.length-1].x ||
                y.get(y.size()-1) != path[path.length-1].y ||
                v.get(v.size()-1) != vels[vels.length-1]){
            x.add(path[path.length-1].x);
            y.add(path[path.length-1].y);
            v.add(vels[vels.length-1]);
        }
    }

    /**
     * Calculates the target thetas for each point along the path
     * @param startIdx index to start calculating from
     */
    public static void calculateTheta(int startIdx){
        for(int i = startIdx; i < x.size()-1; i++){
            double x0 = x.get(i);
            double y0 = y.get(i);
            double x1 = x.get(i+1);
            double y1 = y.get(i+1);
            theta.add(Math.atan2(y1-y0, x1-x0));
        }
        theta.add(theta.get(theta.size()-1));
    }

    /**
     * Calculates the target omega for each point along the path
     * @param startIdx index to start calculating from
     */
    public static void calculateOmega(int startIdx){
        for(int i = startIdx; i < x.size()-1; i++){
            double theta0 = theta.get(i);
            double theta1 = theta.get(i+1);
            omega.add((theta1-theta0)/0.01);
        }
        omega.add(0.0);
    }
}
