
#ifndef METRICUPGRADE_H
#define METRICUPGRADE_H

#include <opencv2\core\core.hpp>

#define NUM_VIEWS 8

int MetricUpgrade(double *ProjectionM, double *Q);
int FindHfromQ(double *Q, cv::Mat& H);

#endif