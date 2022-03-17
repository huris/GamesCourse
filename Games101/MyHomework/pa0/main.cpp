#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

#define PI acos(-1.0)

inline float Degree2Rad(float Degree) {
    return Degree / 180.0 * PI;
}

int main(){

    // 给定一个点 P =(2,1), 将该点绕原点先逆时针旋转 45◦，再平移 (1,2)
    // 计算出变换后点的坐标(要求用齐次坐标进行计算)。
    Eigen::MatrixXd P(3, 1);    // 确定点的向量大小
    Eigen::MatrixXd Transform(3, 3);  // 确定变换矩阵的大小

    P << 2.0f, 1.0f, 1.0f;

    Transform << cos(Degree2Rad(45.0f)), -sin(Degree2Rad(45.0f)), 1.0f,
                 sin(Degree2Rad(45.0f)),  cos(Degree2Rad(45.0f)), 2.0f,
                 0.0f, 0.0f, 1.0f;

    P = Transform * P;

    std::cout << P << std::endl;

    return 0;
}