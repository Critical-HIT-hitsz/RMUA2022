#include "visualize.h"
Eigen:: VectorXd get_rbox_point(double x,double y,double sinA,double cosA,int y_rows)
{       
    // 这个地方长宽有点小bug 但能用
        double width = 60;
        double height = 45;
        
        y = y_rows - y;
        double  x1 = x - 0.5 * width;
        double  y1 = y - 0.5 * height;

        double x0 = x + 0.5 * width;
        double y0 = y1;

        double x2 = x1;
        double y2 = y + 0.5 * height;
        double x3 = x0;
        double y3 = y2;

        Eigen::VectorXd rbox_point(8);
        double x0n = (x0 - x) * cosA - (y0-y) * sinA + x;
        double y0n = (x0 - x) * sinA + (y0-y) * cosA + y;
        rbox_point(0) = x0n;
        rbox_point(1) = y_rows- y0n;

        double x1n = (x1 - x) * cosA - (y1-y) * sinA + x;
        double y1n = (x1 - x) * sinA + (y1-y) * cosA + y;
        rbox_point(2) = x1n;
        rbox_point(3) = y_rows-y1n;

        double x2n = (x2 - x) * cosA - (y2-y) * sinA + x;
        double y2n = (x2 - x) * sinA + (y2-y) * cosA + y;
        rbox_point(4) = x2n;
        rbox_point(5) = y_rows - y2n;


        double x3n = (x3 - x) * cosA - (y3-y) * sinA +  x; 
        double y3n = (x3 - x) * sinA + (y3-y) * cosA + y;
        rbox_point(6) = x3n;
        rbox_point(7) = y_rows -y3n;

        return rbox_point;

}