#ifndef VISUAL_TOOL_H
#define VISUAL_TOOL_H


#include "rsort.h"

Eigen::VectorXd get_rbox_point(double x,double y,double sinA,double cosA,int y_rows);
Eigen::VectorXd get_rbox_point(detbox detection,int y_rows);

#endif
