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

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);

using namespace std;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

using PoinT = pcl::PointXYZ;
using PointN = pcl::Normal;
float radius = 0.03f;



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
	pcl::visualization::PCLVisualizer viewer("cloud");
	viewer.addPointCloud<PoinT>(cloud, "cloud");
	viewer.spin();

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
	vector<Vector3f> pointToKeypointVector;

	//����x��
	for (size_t i = 0; i < cloud->size(); i++)
	{
		if (kdtree.radiusSearch(cloud->points[i], radius, pointidxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			for (size_t j = 0; j < pointidxRadiusSearch.size(); j++)
			{
				������
			}
		}
	}


	

	system("pause");

	return(0);
}