//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        if(u < 0) u = 0; if(u > 1) u = 1;
        if(v < 0) v = 0; if(v > 1) v = 1;
        auto u_img = u * (width - 1);
        auto v_img = (1 - v) * (height - 1);
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = v * height;

        auto u00X = floor(u_img), u00Y = floor(v_img);
        auto u10X = ceil(u_img), u10Y = floor(v_img);
        auto u01X = floor(u_img), u01Y = ceil(v_img);
        auto u11X = ceil(u_img), u11Y = ceil(v_img);

        auto ColorDown = Lerp(u_img - u00X, getColor(u00X / width, u00Y / height), getColor(u10X / width, u10Y / height));
        auto ColorUpon = Lerp(u_img - u01X, getColor(u01X / width, u01Y / height), getColor(u11X / width, u11Y / height));

        auto color = Lerp(v_img - u00Y, ColorDown, ColorUpon);

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f Lerp(float coefficient, Eigen::Vector3f a, Eigen::Vector3f b){
        return a + coefficient * (b - a);
    }

};
#endif //RASTERIZER_TEXTURE_H
