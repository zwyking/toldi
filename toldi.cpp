#include <iostream>//标准C++库中的输入输出类相关头文件。  
#include <stdlib.h>
#include <vector>
#include <pcl/io/io.h>  
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。  
#include <pcl/io/ply_io.h>  
#include <pcl/point_types.h> //PCL中支持的点类型头文件。 
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/impl/3dsc.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);

using namespace std;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

using PoinT = pcl::PointXYZ;
using PointN = pcl::Normal;
float radius = 0.9f;
//float radius = 0.03f;

typedef struct
{
	float w1;
	float w2;
}point_weight;

int TOLDI_description(string &path, vector<vector<float>> & TOLDIfeatures, pcl::PointCloud<PoinT>::Ptr &cloud);
void Match(vector<vector<float>> & source, vector<vector<float>> & tes, vector<vector<int>> &registration);
void Matrix_Computer(vector<vector<int>> &registration, pcl::PointCloud<PoinT>::Ptr &source_cloud, pcl::PointCloud<PoinT>::Ptr &test_cloud);

int main()
{
	int flag;
	vector<vector<float>> feature1, feature2;
	string path_1 = "Bologna_Retrieval\\Models\\Chinese_dragon.ply";
	string path_2 = "Bologna_Retrieval\\Scenes\\original\\Scene2.ply";
	pcl::PointCloud<PoinT>::Ptr source_cloud(new pcl::PointCloud<PoinT>);
	pcl::PointCloud<PoinT>::Ptr test_cloud(new pcl::PointCloud<PoinT>);

	if (TOLDI_description(path_1, feature1, source_cloud) == -1)
	{
		return -1;
	}
	if (TOLDI_description(path_2, feature2, test_cloud) == -1)
	{
		return -1;
	}
	vector<vector<int>> registration;
	Match(feature1, feature2, registration);
	Matrix_Computer(registration, source_cloud, test_cloud);
	system("pause");
	return 0;
}

int TOLDI_description(string &path, vector<vector<float>> & TOLDIfeatures, pcl::PointCloud<PoinT>::Ptr &cloud)
{
	/***读取ply文件***/
	if (pcl::io::loadPLYFile<PoinT>(path , *cloud) == -1)
	{
		PCL_ERROR("Couldn't read the pcd file \n"); //文件不存在时，返回错误，终止程序。

		system("pause");

		return (-1);
	}
	cout << cloud->size() << endl;


	/***降采样***/
	// 创建滤波对象
	pcl::VoxelGrid<PoinT> filter;
	filter.setInputCloud(cloud);
	// 设置体素栅格的大小
	filter.setLeafSize(0.004f, 0.004f, 0.004f);
	filter.filter(*cloud);
	cout << cloud->size() << endl;

<<<<<<< HEAD
	
	/***点云可视化***/
	/*pcl::visualization::PCLVisualizer viewer("cloud");
	viewer.addPointCloud<PoinT>(cloud, "cloud");
	viewer.spin();*/
=======

	/***点云可视化***/
	/*pcl::visualization::PCLVisualizer viewer("cloud");
	viewer.addPointCloud<PoinT>(cloud, "cloud");
	viewer.spinOnce();*/
>>>>>>> 59b67a6e6141cab93032016c12f82484949e5a60


	/***建立kd树，计算点云中的LPR***/
	//计算法向量(z轴)
	pcl::NormalEstimation<PoinT, PointN> normal_estimation;
	normal_estimation.setInputCloud(cloud);
	pcl::search::KdTree<PoinT>::Ptr tree(new pcl::search::KdTree<PoinT>);
	normal_estimation.setSearchMethod(tree);
	pcl::PointCloud<PointN>::Ptr cloud_normals(new pcl::PointCloud<PointN>);
	normal_estimation.setRadiusSearch(radius);
	normal_estimation.compute(*cloud_normals);

	//构建kd搜索树
	pcl::KdTreeFLANN<PoinT> kdtree;
	kdtree.setInputCloud(cloud);

	vector<int> pointidxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;
	vector<Vector3f> pointToKeypointVector;//储存中心点到搜索点的3维向量
	vector<Vector3f> zAxisNromal;//储存z轴方向
	Vector3f temp, zaxis_vector;
	vector<point_weight> pointXaxisWeight;//储存L2正则化的权重系数
	point_weight temp_weight;
	vector<Vector3f> xAxisNormal;//储存x轴方向
	Vector3f xaxis_vector;
	vector<Vector3f> yAxisNormal;//储存y轴方向
	Vector3f yaxis_vector;
	pcl::PointCloud<PoinT>::Ptr LRF_cloud(new pcl::PointCloud<PoinT>);//储存LRF坐标系下的坐标
	pcl::PointCloud<PoinT>::Ptr LRF_projected(new pcl::PointCloud<PoinT>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::ProjectInliers<PoinT> proj;
	//pcl::VoxelGrid<PoinT> filter_2D;
	//filter_2D.setLeafSize(radius / 10, radius / 10, radius / 10);
	vector<float> histogram;
	int index_x, index_y;
	vector<vector<float>> TOLDIfeatures;

	//计算LRF坐标系及TOLDI算子
	for (size_t i = 0; i < cloud->size(); i++)
	{
		if (kdtree.radiusSearch(cloud->points[i], radius, pointidxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			zaxis_vector << cloud_normals->points[i].normal_x, cloud_normals->points[i].normal_y, cloud_normals->points[i].normal_z;
			zAxisNromal.push_back(zaxis_vector);
			xaxis_vector << 0, 0, 0;
			for (size_t j = 0; j < pointidxRadiusSearch.size(); j++)
			{
				/*temp.x << cloud->points[pointidxRadiusSearch[j]].x - cloud->points[i].x;
				temp.y << cloud->points[pointidxRadiusSearch[j]].y - cloud->points[i].y;
				temp.z << cloud->points[pointidxRadiusSearch[j]].z - cloud->points[i].z;*/
				temp << cloud->points[pointidxRadiusSearch[j]].x - cloud->points[i].x, cloud->points[pointidxRadiusSearch[j]].y - cloud->points[i].y, cloud->points[pointidxRadiusSearch[j]].z - cloud->points[i].z;
				temp_weight.w1 = pow(radius - sqrt(pow(temp[0], 2) + pow(temp[1], 2) + pow(temp[2], 2)), 2);
				temp_weight.w2 = pow(temp.dot(zaxis_vector), 2);
				temp = temp - (temp.dot(zaxis_vector))*zaxis_vector;
				pointXaxisWeight.push_back(temp_weight);
				pointToKeypointVector.push_back(temp);
			}
			for (size_t k = 0; k < pointidxRadiusSearch.size(); k++)
			{
				xaxis_vector = xaxis_vector + pointXaxisWeight[k].w1*pointXaxisWeight[k].w2*pointToKeypointVector[k];
			}
			xaxis_vector = xaxis_vector / sqrt(pow(xaxis_vector[0], 2) + pow(xaxis_vector[1], 2) + pow(xaxis_vector[2], 2));
			xAxisNormal.push_back(xaxis_vector);
			yaxis_vector = zaxis_vector.cross(xaxis_vector);
			yAxisNormal.push_back(yaxis_vector);

			LRF_cloud->resize(pointidxRadiusSearch.size());
			//计算LRF新坐标
			for (size_t k = 0; k < pointidxRadiusSearch.size(); k++)
			{
				LRF_cloud->points[k].x = pointToKeypointVector[k].dot(xaxis_vector);
				LRF_cloud->points[k].y = pointToKeypointVector[k].dot(yaxis_vector);
				LRF_cloud->points[k].z = pointToKeypointVector[k].dot(zaxis_vector);
			}

			////投影到X-Y平面
			//coefficients->values.resize(4);
			//coefficients->values[0] = 0;
			//coefficients->values[1] = 0;
			//coefficients->values[2] = 1;
			//coefficients->values[3] = 0;

			//proj.setModelType(pcl::SACMODEL_PLANE);
			//proj.setInputCloud(LRF_cloud);
			//proj.setModelCoefficients(coefficients);
			//proj.filter(*LRF_projected);

			histogram.insert(histogram.begin(), 1200, 2);
			for (size_t k = 0; k < pointidxRadiusSearch.size(); k++)
			{
				index_x = floor((radius + LRF_cloud->points[k].x) * 10 / radius);
				index_y = floor((radius + LRF_cloud->points[k].y) * 10 / radius);
				//cout <<  LRF_cloud->points[k].y << endl;
				if (((radius - LRF_cloud->points[k].z) / (2 * radius)) < histogram[index_x + index_y * 20])
				{
					histogram[index_x + index_y * 20] = (radius - LRF_cloud->points[k].z) / (2 * radius);
				}
			}
			for (size_t k = 0; k < pointidxRadiusSearch.size(); k++)
			{
				index_x = floor((radius + LRF_cloud->points[k].x) * 10 / radius);
				index_y = floor((radius + LRF_cloud->points[k].z) * 10 / radius);
				//cout <<  LRF_cloud->points[k].y << endl;
				if (((radius - LRF_cloud->points[k].y) / (2 * radius)) < histogram[index_x + index_y * 20])
				{
					histogram[index_x + index_y * 20 + 400] = (radius - LRF_cloud->points[k].y) / (2 * radius);
				}
<<<<<<< HEAD
			}
			for (size_t k = 0; k < pointidxRadiusSearch.size(); k++)
			{
				index_x = floor((radius + LRF_cloud->points[k].y) * 10 / radius);
				index_y = floor((radius + LRF_cloud->points[k].z) * 10 / radius);
				//cout <<  LRF_cloud->points[k].y << endl;
				if (((radius - LRF_cloud->points[k].x) / (2 * radius)) < histogram[index_x + index_y * 20])
				{
					histogram[index_x + index_y * 20 + 800] = (radius - LRF_cloud->points[k].x) / (2 * radius);
				}
			}

			/*for (size_t k = 0; k < 1200; k++)
			{
				cout << histogram[k] << endl;
=======
			}
			for (size_t k = 0; k < pointidxRadiusSearch.size(); k++)
			{
				index_x = floor((radius + LRF_cloud->points[k].y) * 10 / radius);
				index_y = floor((radius + LRF_cloud->points[k].z) * 10 / radius);
				//cout <<  LRF_cloud->points[k].y << endl;
				if (((radius - LRF_cloud->points[k].x) / (2 * radius)) < histogram[index_x + index_y * 20])
				{
					histogram[index_x + index_y * 20 + 800] = (radius - LRF_cloud->points[k].x) / (2 * radius);
				}
			}

			/*for (size_t k = 0; k < 1200; k++)
			{
			cout << histogram[k] << endl;
>>>>>>> 59b67a6e6141cab93032016c12f82484949e5a60
			}*/
			TOLDIfeatures.push_back(histogram);
			histogram.clear();
			pointToKeypointVector.clear();
			pointXaxisWeight.clear();
			pointRadiusSquaredDistance.clear();
			pointidxRadiusSearch.clear();
		}
		cout << i << endl;
	}

<<<<<<< HEAD
	
	system("pause");

=======
>>>>>>> 59b67a6e6141cab93032016c12f82484949e5a60
	return(0);
}

void Match(vector<vector<float>> & source, vector<vector<float>> & test, vector<vector<int>> &registration)
{
	vector<int> dual_points;
	float similarity;//计算两个特征之间的相似度
	float min_similarity;
	for (int i = 0;i < source.size();i++)
	{
		dual_points.resize(3);
		dual_points[0] = i;
		dual_points[1] = 0;
		min_similarity = INFINITE;
		for (int j = 0;j < test.size();j++)
		{
			similarity = 0;
			for (size_t k = 0;k < 1200;k++)
			{
				similarity = similarity + pow(test[j][k] - source[i][k], 2);
			}
			if (similarity < min_similarity)
			{
				min_similarity = similarity;
				dual_points[1] = j;
			}
		}
		dual_points[2] = min_similarity;
		registration.push_back(dual_points);
	}

	dual_points.resize(3);
	for (size_t i = 0;i < registration.size()-1;i++)
	{
		for (size_t j = i + 1;j < registration.size();j++)
		{
			if (registration[i][2] > registration[j][2])
			{
				dual_points = registration[i];
				registration[i] = registration[j];
				registration[j] = dual_points;
			}
		}
	}
}

void Matrix_Computer(vector<vector<int>> &registration, pcl::PointCloud<PoinT>::Ptr &source_cloud, pcl::PointCloud<PoinT>::Ptr &test_cloud)
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>()); 		
	cloud_in->height = 1;	
	cloud_in->is_dense = false;
	cloud_in->resize(5); 

	cloud_out->height = 1;	
	cloud_out->is_dense = false;	
	cloud_out->resize(5);

	for (int i = 0;i < cloud_in->points.size();i++)
	{
		cloud_out->points[i].x = test_cloud->points[registration[i][1]].x;
		cloud_out->points[i].y = test_cloud->points[registration[i][1]].y;
		cloud_out->points[i].z = test_cloud->points[registration[i][1]].z;
		cloud_in->points[i].x = source_cloud->points[registration[i][0]].x;
		cloud_in->points[i].y = source_cloud->points[registration[i][0]].y;
		cloud_in->points[i].z = source_cloud->points[registration[i][0]].z;
	}

	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation2;
	TESVD.estimateRigidTransformation(*cloud_in, *cloud_out, transformation2);
/*
	transformation2(0, 0) = 0.528868;
	transformation2(0, 1) = 0.17417;
	transformation2(0, 2) = -0.83064;
	transformation2(0, 3) = 0.107351;
	transformation2(1, 0) = -0.130978;
	transformation2(1, 1) = 0.983741;
	transformation2(1, 2) = 0.122878;
	transformation2(1, 3) = 0.091175;
	transformation2(2, 0) = 0.838536;
	transformation2(2, 1) = 0.04381;
	transformation2(2, 2) = 0.543082;
	transformation2(2, 3) = 0.027941;*/

	pcl::PointCloud<PoinT>::Ptr transformed_cloud(new pcl::PointCloud<PoinT>);
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transformation2);
	//pcl::transformPointCloud(*test_cloud, *transformed_cloud, transformation2);


	cout << "开始RANSAC" << endl;
	/***RANSAC***/
	float distance;
	float distance_threshold = 0.001f;
	float points_threshold = source_cloud->size()*0.85;
	vector<int> dual_points;
	vector<vector<int>> inside_points;
	dual_points.resize(2);
	srand((unsigned)time(NULL));
	int k;
	for (size_t p = 0;p < 1000;p++)
	{
		for (size_t i = 0;i < registration.size();i++)
		{
			distance = 0;
			distance = distance + pow(transformed_cloud->points[registration[i][0]].x - test_cloud->points[registration[i][1]].x, 2);
			distance = distance + pow(transformed_cloud->points[registration[i][0]].y - test_cloud->points[registration[i][1]].y, 2);
			distance = distance + pow(transformed_cloud->points[registration[i][0]].z - test_cloud->points[registration[i][1]].z, 2);
			if (distance < distance_threshold)
			{
				dual_points[0] = registration[i][0];
				dual_points[1] = registration[i][1];
				inside_points.push_back(dual_points);
			}
		}
		cout << inside_points.size() << endl;
		if ((inside_points.size() > points_threshold) && (p > 500))
		{
			cout <<" ---- "<< p << endl;
			break;
		}
		for (size_t j = 0;j < cloud_in->points.size();j++)
		{
			k = rand() % inside_points.size();
			cloud_out->points[j].x = test_cloud->points[inside_points[k][1]].x;
			cloud_out->points[j].y = test_cloud->points[inside_points[k][1]].y;
			cloud_out->points[j].z = test_cloud->points[inside_points[k][1]].z;
			cloud_in->points[j].x = source_cloud->points[inside_points[k][0]].x;
			cloud_in->points[j].y = source_cloud->points[inside_points[k][0]].y;
			cloud_in->points[j].z = source_cloud->points[inside_points[k][0]].z;
		}
		transformed_cloud->clear();
		inside_points.clear();
		TESVD.estimateRigidTransformation(*cloud_in, *cloud_out, transformation2);
		pcl::transformPointCloud(*source_cloud, *transformed_cloud, transformation2);
	}


	std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << std::endl;	
	printf("\n");	
	printf("    | %6.3f %6.3f %6.3f | \n", transformation2(0, 0), transformation2(0, 1), transformation2(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", transformation2(1, 0), transformation2(1, 1), transformation2(1, 2));	
	printf("    | %6.3f %6.3f %6.3f | \n", transformation2(2, 0), transformation2(2, 1), transformation2(2, 2));	
	printf("\n");	
	printf("t = < %0.3f, %0.3f, %0.3f >\n", transformation2(0, 3), transformation2(1, 3), transformation2(2, 3));


	cout << endl;
	cout << source_cloud->size() << endl;
	cout << test_cloud->size() << endl;


	
	// 可视化
	// 可视化将原始点云显示为白色，变换后的点云为红色，还设置了坐标轴、背景颜色、点显示大小
	printf("\nPoint cloud colors :  white  = original point cloud\n"
		"                        red  = transformed point cloud\n");
	pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

	// 为点云定义 R,G,B 颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(test_cloud, 255, 255, 255);
	// 输出点云到查看器，使用颜色管理
	viewer.addPointCloud(test_cloud, source_cloud_color_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // 红
	viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // 设置背景为深灰
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	//viewer.setPosition(800, 400); // 设置窗口位置

	while (!viewer.wasStopped())
	{// 在按下 "q" 键之前一直会显示窗口
		viewer.spinOnce();
	}
}