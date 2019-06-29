#ifndef MYTOOLS
#define MYTOOLS

#define NOMINMAX

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <igl/vertex_triangle_adjacency.h>
#include "nanoflann.hpp"
#include <igl/fit_plane.h>
#include <Eigen/SVD>
void get_axis(Eigen::MatrixXd & axis);


void calculate_vertex_normal(
	Eigen::MatrixXd const & V,
	Eigen::MatrixXi const & F,
	Eigen::MatrixXd const & FN,
	Eigen::MatrixXd & out_VN);

void calculate_vertex_normal_flann(
	Eigen::MatrixXd const & V, 
	Eigen::MatrixXi const & F,  
	Eigen::MatrixXd & out_VN);

void roughlyAlign(
		Eigen::MatrixXd & m2_V,
		double xAngle,
        double yAngle,
        double zAngle,
        double xT,
        double yT,
        double zT);

void ICP_p2Point(
                 Eigen::MatrixXd & Q,
                 Eigen::MatrixXd const P);

void addGaussNoise(
                     Eigen::MatrixXd & M, double scale);

void subsampling(
                 Eigen::MatrixXd & M,
                 int percentageOfSamples);

void rejectPoints(
                  Eigen::MatrixXd & Q,
                  Eigen::MatrixXd & NNs);

Eigen::Matrix3d getRotationMatrix(
                                  float xAngle,
                                  float yAngle,
                                  float zAngle);

void ICP_p2Plane(
                 Eigen::MatrixXd & Q,
                 Eigen::MatrixXd const P);

Eigen::Matrix4d point2planeTransform(
                                     Eigen::MatrixXd & Q,
                                     Eigen::MatrixXd const P,
                                     Eigen::MatrixXd & NNs_Normals);

Eigen::MatrixXd findNN(
            Eigen::MatrixXd Q,
            Eigen::MatrixXd P,
            size_t numberOfNN);





#endif


