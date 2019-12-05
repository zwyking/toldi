#include <iostream>//��׼C++���е�������������ͷ�ļ���  
#include <vector>
#include <pcl/io/io.h>  
#include <pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���  
#include <pcl/io/ply_io.h>  
#include <pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ��� 
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/impl/3dsc.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);

using namespace std;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

using PoinT = pcl::PointXYZ;
using PointN = pcl::Normal;
float radius = 0.015f;
//float radius = 0.03f;

typedef struct
{
	float w1;
	float w2;
}point_weight;


int main()
{
	/***��ȡpcd�ļ�***/
	pcl::PointCloud<PoinT>::Ptr cloud(new pcl::PointCloud<PoinT>);
	if (pcl::io::loadPCDFile("bunny.pcd", *cloud) == -1)
	{
		PCL_ERROR("Couldn't read the pcd file \n"); //�ļ�������ʱ�����ش�����ֹ����

		system("pause");

		return (-1);
	}
	cout << cloud->size()<<endl;


	/***������***/
	// �����˲�����
	pcl::VoxelGrid<PoinT> filter;
	filter.setInputCloud(cloud);
	// ��������դ��Ĵ�С
	filter.setLeafSize(0.005f, 0.005f, 0.005f);
	filter.filter(*cloud);
	cout << cloud->size() << endl;

	
	/***���ƿ��ӻ�***/
	/*pcl::visualization::PCLVisualizer viewer("cloud");
	viewer.addPointCloud<PoinT>(cloud, "cloud");
	viewer.spin();*/


	/***����kd������������е�LPR***/
	//���㷨����(z��)
	pcl::NormalEstimation<PoinT, PointN> normal_estimation;
	normal_estimation.setInputCloud(cloud);
	pcl::search::KdTree<PoinT>::Ptr tree(new pcl::search::KdTree<PoinT>);
	normal_estimation.setSearchMethod(tree);
	pcl::PointCloud<PointN>::Ptr cloud_normals(new pcl::PointCloud<PointN>);
	normal_estimation.setRadiusSearch(radius);
	normal_estimation.compute(*cloud_normals);

	//����kd������
	pcl::KdTreeFLANN<PoinT> kdtree;
	kdtree.setInputCloud(cloud);

	vector<int> pointidxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;
	vector<Vector3f> pointToKeypointVector;//�������ĵ㵽�������3ά����
	vector<Vector3f> zAxisNromal;//����z�᷽��
	Vector3f temp, zaxis_vector;
	vector<point_weight> pointXaxisWeight;//����L2���򻯵�Ȩ��ϵ��
	point_weight temp_weight;
	vector<Vector3f> xAxisNormal;//����x�᷽��
	Vector3f xaxis_vector;
	vector<Vector3f> yAxisNormal;//����y�᷽��
	Vector3f yaxis_vector;
	pcl::PointCloud<PoinT>::Ptr LRF_cloud(new pcl::PointCloud<PoinT>);//����LRF����ϵ�µ�����
	pcl::PointCloud<PoinT>::Ptr LRF_projected(new pcl::PointCloud<PoinT>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::ProjectInliers<PoinT> proj;
	//pcl::VoxelGrid<PoinT> filter_2D;
	//filter_2D.setLeafSize(radius / 10, radius / 10, radius / 10);
	vector<float> histogram;
	int index_x, index_y;
	vector<vector<float>> TOLDIfeatures;

	//����LRF����ϵ��TOLDI����
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
			//����LRF������
			for (size_t k = 0; k < pointidxRadiusSearch.size(); k++)
			{
				LRF_cloud->points[k].x = pointToKeypointVector[k].dot(xaxis_vector);
				LRF_cloud->points[k].y = pointToKeypointVector[k].dot(yaxis_vector);
				LRF_cloud->points[k].z = pointToKeypointVector[k].dot(zaxis_vector);
			}

			////ͶӰ��X-Yƽ��
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

	
	system("pause");

	return(0);
}