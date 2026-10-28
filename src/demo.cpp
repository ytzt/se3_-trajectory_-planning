#include "SE3Interpolators.hpp"
#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// #define Base_INTERP 
#define BAZIER_INTERP 

// 插值器类型定义
using InterpolatorFunc = std::function<Eigen::Isometry3d(
    const Eigen::Isometry3d&, 
    const Eigen::Isometry3d&, 
    const Eigen::Isometry3d&, 
    const Eigen::Isometry3d&, 
    double)>;

// 基础线性插值器
Eigen::Isometry3d baseInterpolator(
    const Eigen::Isometry3d& T0, 
    const Eigen::Isometry3d& T1, 
    double t) 
{
    Sophus::SE3d sophus_T0(T0.rotation(), T0.translation());
    Sophus::SE3d sophus_T1(T1.rotation(), T1.translation());
    Sophus::SE3d interp = SE3BaseInterp::interpolate(sophus_T0, sophus_T1, t);
    
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result = interp.matrix();
    return result;
}

// 贝塞尔曲线插值器
Eigen::Isometry3d bezierInterpolator(
    const Eigen::Isometry3d& T0, 
    const Eigen::Isometry3d& T1, 
    const Eigen::Isometry3d& T2, 
    const Eigen::Isometry3d& T3, 
    double t) 
{
    std::vector<Sophus::SE3d> control_points = {
        Sophus::SE3d(T0.rotation(), T0.translation()),
        Sophus::SE3d(T1.rotation(), T1.translation()),
        Sophus::SE3d(T2.rotation(), T2.translation()),
        Sophus::SE3d(T3.rotation(), T3.translation())
    };
    
    SE3BezierInterpolator bezier(control_points);
    Sophus::SE3d result_se3 = bezier.evaluate(t);
    
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    // result.linear() = result_se3.rotationMatrix();
    // result.translation() = result_se3.translation();
    result = result_se3.matrix();
	return result;
    // return result_se3.matrix();
}

int test() {
    // 创建控制点 (示例使用简单变换)

    std::vector<Sophus::SE3d> control_points;
    control_points.emplace_back(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));
    control_points.emplace_back(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1, 0, 0));
    control_points.emplace_back(
        Sophus::SO3d::exp(Eigen::Vector3d(0, 0, M_PI/2)), 
        Eigen::Vector3d(1, 1, 0)
    );
    control_points.emplace_back(
        Sophus::SO3d::exp(Eigen::Vector3d(0, 0, M_PI)), 
        Eigen::Vector3d(0, 1, 0)
    );

    // 1. 基础插值测试
    Sophus::SE3d T0 = control_points[0];
    Sophus::SE3d T1 = control_points[3];
    Sophus::SE3d interp = SE3BaseInterp::interpolate(T0, T1, 0.5);
    std::cout << "Base interpolation result:\n" << interp.matrix() << "\n\n";

    // 2. 贝塞尔曲线测试
    SE3BezierInterpolator bezier(control_points);
    Sophus::SE3d bezier_pose = bezier.evaluate(0.7);
    std::cout << "Bezier result:\n" << bezier_pose.matrix() << "\n\n";

    // 3. B样条曲线测试
    SE3BSplineInterpolator bspline(control_points, 3);
    Sophus::SE3d bspline_pose = bspline.evaluate(0.4);
    std::cout << "B-spline result:\n" << bspline_pose.matrix() << "\n";

    return 0;
}

int main(){
    pcl::PointCloud<pcl::PointXYZI> after,pose,temp,out,cp_in,cp_out,cp_save;

    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/90, Eigen::Vector3d(0,0,1)).toRotationMatrix();
	Eigen::Vector3d t(1,0,0);
	Eigen::Isometry3d temp_pose1;
	Eigen::Isometry3d temp_pose2;
	Eigen::Isometry3d temp_pose3;
	Eigen::Isometry3d temp_pose4;
	Eigen::Isometry3d temp_pose5;
	Eigen::Isometry3d temp_pose6;
	Eigen::Isometry3d temp_pose7;
	pcl::PointXYZI temp1,temp2,temp3;
	temp_pose1.setIdentity();
	temp_pose1.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0,0,1)).toRotationMatrix());
	temp_pose1.translate(Eigen::Vector3d(0,0,0.5));
	std::cout<<"temp_pose1:"<<std::endl;
	std::cout<<temp_pose1.matrix()<<std::endl;
	temp_pose2.setIdentity();
	temp_pose2.rotate(Eigen::AngleAxisd(1.5*M_PI/4, Eigen::Vector3d(0,.1,1).normalized()).toRotationMatrix());
	temp_pose2.translate(Eigen::Vector3d(0,0,1));
	std::cout<<"temp_pose2:"<<std::endl;
	std::cout<<temp_pose2.matrix()<<std::endl;
	temp_pose3.setIdentity();
	temp_pose3.rotate(Eigen::AngleAxisd(2.2*M_PI/4, Eigen::Vector3d(0,.3,1).normalized()).toRotationMatrix());
	temp_pose3.translate(Eigen::Vector3d(0,0,2.3));
	std::cout<<"temp_pose3:"<<std::endl;
	std::cout<<temp_pose3.matrix()<<std::endl;
	temp_pose4.setIdentity();
	temp_pose4.rotate(Eigen::AngleAxisd(3.2*M_PI/4, Eigen::Vector3d(.5,0,1).normalized()).toRotationMatrix());
	temp_pose4.translate(Eigen::Vector3d(0,0,3.7));
	std::cout<<"temp_pose4:"<<std::endl;
	std::cout<<temp_pose4.matrix()<<std::endl;
	temp_pose5.setIdentity();
	temp_pose5.rotate(Eigen::AngleAxisd(2.1*M_PI/4, Eigen::Vector3d(.2,0,1).normalized()).toRotationMatrix());
	temp_pose5.translate(Eigen::Vector3d(0,1,3.7));
	std::cout<<"temp_pose5:"<<std::endl;
	std::cout<<temp_pose5.matrix()<<std::endl;
	temp_pose6.setIdentity();
	temp_pose6.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0,-0.2,1).normalized()).toRotationMatrix());
	temp_pose6.translate(Eigen::Vector3d(1,1,3.7));
	std::cout<<"temp_pose6:"<<std::endl;
	std::cout<<temp_pose6.matrix()<<std::endl;
	temp_pose7.setIdentity();
	temp_pose7.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0.4,0,1).normalized()).toRotationMatrix());
	temp_pose7.translate(Eigen::Vector3d(1,1.5,3.7));
	std::cout<<"temp_pose7:"<<std::endl;
	std::cout<<temp_pose7.matrix()<<std::endl;
	// Sophus::SE3d a(Eigen::AngleAxisd(0, Eigen::Vector3d(0,0,1)).toRotationMatrix(),Eigen::Vector3d(0,0,0)),
	// 		b(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0,0.2,1)).toRotationMatrix(),Eigen::Vector3d(1,2,1)); //错误，轴不是单位向量
	// std::std::cout << "b rotation matrix:\n" << b.rotationMatrix() << std::std::endl;
	//std::cout<<"curr: a->b:"<<std::endl<<fromAtoB(a,b)<<std::endl;
	int intens = 0;
	temp2.y = 0;
	temp2.z = 0;
	for (double j = 0; j < 100; ++j) {
		temp2.x = j/100.0;
		temp2.y = 0;
		temp2.z = 0;
		temp2.intensity = 0.5;
		temp.push_back(temp2);
	}
	for (double j = 0; j < 100; ++j) {
		temp2.x = 0;
		temp2.y = j/100.0;
		temp2.intensity = 2;
		temp.push_back(temp2);
	}
	for (double j = 0; j < 100; ++j) {
		temp2.x = 0;
		temp2.y = 0;
		temp2.z = j/100.0;
		temp2.intensity = 3;
		temp.push_back(temp2);
	}
	cp_in = temp;
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_in[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose1.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose2.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose3.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose4.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose5.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose6.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose7.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::io::savePCDFile("cp_save.pcd",cp_save);
	
	
	// for (double j = 0; j < 300; ++j) {
	// 	pose.push_back(temp[j]);
	// }
	pose += cp_save;          // ← 整云追加，等价于 insert(end, begin, end)
    double sample_number = 50;
    
    #ifdef Base_INTERP
   
	for (double i = 0; i < sample_number; ++i) {
		Eigen::Isometry3d tf;
		Eigen::Matrix4d mat;
		tf = baseInterpolator(temp_pose1,temp_pose2, i/sample_number);
		std::cout<<i<<std::endl<<tf.matrix()<<std::endl;
		mat = tf.matrix();
		temp1.x = mat(0,3);
		temp1.y = mat(1,3);
		temp1.z = mat(2,3);
		temp1.intensity = intens;
		intens++;
		pcl::transformPointCloud(temp,out,mat);
		for (double j = 0; j < 300; ++j) {
			pose.push_back(out[j]);
		}
		
		after.push_back(temp1);
	}
	for (double i = 0; i < sample_number; ++i) {
		Eigen::Isometry3d tf;
		Eigen::Matrix4d mat;
		tf = baseInterpolator(temp_pose2,temp_pose3,i/sample_number);
		std::cout<<i<<std::endl<<tf.matrix()<<std::endl;
		mat = tf.matrix();
		temp1.x = mat(0,3);
		temp1.y = mat(1,3);
		temp1.z = mat(2,3);
		temp1.intensity = intens;
		intens++;
		pcl::transformPointCloud(temp,out,mat);
		for (double j = 0; j < 300; ++j) {
			pose.push_back(out[j]);
		}
		after.push_back(temp1);
	}
	for (double i = 0; i < sample_number; ++i) {
		Eigen::Isometry3d tf;
		Eigen::Matrix4d mat;
		tf = baseInterpolator(temp_pose3,temp_pose4,i/sample_number);
		std::cout<<i<<std::endl<<tf.matrix()<<std::endl;
		mat = tf.matrix();
		temp1.x = mat(0,3);
		temp1.y = mat(1,3);
		temp1.z = mat(2,3);
		temp1.intensity = intens;
		intens++;
		pcl::transformPointCloud(temp,out,mat);
		for (double j = 0; j < 300; ++j) {
			pose.push_back(out[j]);
		}
		after.push_back(temp1);
	}
	for (double i = 0; i < sample_number; ++i) {
		Eigen::Isometry3d tf;
		Eigen::Matrix4d mat;
		tf = baseInterpolator(temp_pose4,temp_pose5,i/sample_number);
		std::cout<<i<<std::endl<<tf.matrix()<<std::endl;
		mat = tf.matrix();
		temp1.x = mat(0,3);
		temp1.y = mat(1,3);
		temp1.z = mat(2,3);
		pcl::transformPointCloud(temp,out,mat);
		for (double j = 0; j < 300; ++j) {
			pose.push_back(out[j]);
		}
		temp1.intensity = intens;
		intens++;
		after.push_back(temp1);
	}
    for (double i = 0; i < sample_number; ++i) {
		Eigen::Isometry3d tf;
		Eigen::Matrix4d mat;
		tf = baseInterpolator(temp_pose5,temp_pose6,i/sample_number);
		std::cout<<i<<std::endl<<tf.matrix()<<std::endl;
		mat = tf.matrix();
		temp1.x = mat(0,3);
		temp1.y = mat(1,3);
		temp1.z = mat(2,3);
		pcl::transformPointCloud(temp,out,mat);
		for (double j = 0; j < 300; ++j) {
			pose.push_back(out[j]);
		}
		temp1.intensity = intens;
		intens++;
		after.push_back(temp1);
	}
    for (double i = 0; i < sample_number; ++i) {
		Eigen::Isometry3d tf;
		Eigen::Matrix4d mat;
		tf = baseInterpolator(temp_pose6,temp_pose7,i/sample_number);
		std::cout<<i<<std::endl<<tf.matrix()<<std::endl;
		mat = tf.matrix();
		temp1.x = mat(0,3);
		temp1.y = mat(1,3);
		temp1.z = mat(2,3);
		pcl::transformPointCloud(temp,out,mat);
		for (double j = 0; j < 300; ++j) {
			pose.push_back(out[j]);
		}
		temp1.intensity = intens;
		intens++;
		after.push_back(temp1);
	}
    #endif

    #ifdef BAZIER_INTERP
    std::vector<Sophus::SE3d> control_points;
    control_points.reserve(7);  // 预分配空间（可选优化）
    control_points.emplace_back(temp_pose1.rotation(), temp_pose1.translation());
    control_points.emplace_back(temp_pose2.rotation(), temp_pose2.translation());
    control_points.emplace_back(temp_pose3.rotation(), temp_pose3.translation());
    control_points.emplace_back(temp_pose4.rotation(), temp_pose4.translation());
    control_points.emplace_back(temp_pose5.rotation(), temp_pose5.translation());
    control_points.emplace_back(temp_pose6.rotation(), temp_pose6.translation());
    control_points.emplace_back(temp_pose7.rotation(), temp_pose7.translation());
    std::vector<double> ratio{0.1,1,1,1,1,1,0.1};
    // std::vector<double> ratio{1,1,1,1,1,1,1};
    SE3BezierInterpolator bezier(control_points);
    int n = control_points.size() -1;
    for (double i = 0; i < 6.0*sample_number; ++i) {
        Eigen::Isometry3d tf;
		Eigen::Matrix4d mat;
        // Sophus::SE3d result_se3 = bezier.evaluate(i/(6.0*sample_number));
        Sophus::SE3d result_se3 = bezier.RationalBezier(control_points,n,i/(6.0*sample_number),ratio);
        tf = result_se3.matrix();
        mat = tf.matrix();
		temp1.x = mat(0,3);
		temp1.y = mat(1,3);
		temp1.z = mat(2,3);
		pcl::transformPointCloud(temp,out,mat);
		for (double j = 0; j < 300; ++j) {
			pose.push_back(out[j]);
		}
		temp1.intensity = intens;
		intens++;
		after.push_back(temp1);
    }
    
    #endif
	pcl::io::savePCDFile("temp1.pcd",after);
	pcl::io::savePCDFile("pose.pcd",pose);

    return 0;
}