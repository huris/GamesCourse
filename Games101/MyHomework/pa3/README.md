完成：

1. 提交格式正确，包括所有需要的文件。代码可以正常编译、执行。

2. 参数插值: 正确插值颜色、法向量、纹理坐标、位置 (Shading Position) 并将它们传递给 fragment_shader_payload.

    主要在函数`rasterize_triangle`中完成了对颜色，法向量，纹理，视点的插值。同时传给`fragment_shader_payload`。

3. Blinn-phong 反射模型: 正确实现 phong_fragment_shader 对应的 反射模型。

    主要修改了main.cpp函数中的`phong_fragment_shader`，分别计算环境光，漫反射光和镜面反射光，注意环境光只需要算1次。

4. Texture mapping: 将 phong_fragment_shader 的代码拷贝到 texture_fragment_shader, 在此基础上正确实现 Texture Mapping.

    主要修改了main.cpp函数中的`texture_fragment_shader`，将`phong_fragment_shader`中光照计算的代码复制到`texture_fragment_shader`，同时修改了`return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());`

5. Bump mapping 与 Displacement mapping: 正确实现 Bump mapping 与 Displacement mapping.

    a. 求Bump mapping时，主要修改了main.cpp函数中的`bump_fragment_shader`函数，主要按照注释来做，先求TBN矩阵，然后求ln向量，最后两个相乘就得到凹凸贴图了。

    b. 求Displacement mapping，主要修改了main.cpp函数中的`displacement_fragment_shader`函数，将bump mapping 和 blinn-phong 的相关代码复制到该函数中，同时修改一下point的值，最后可以结果。

6. 尝试更多模型: 找到其他可用的.obj 文件，提交渲染结果并 把模型保存在 /models 目录下。

    修改`obj_path`、`loadout`、`texture_path`、`texture_path`，对其他模型进行测试。

7. 线性纹理插值: 使用双线性插值进行纹理采样, 在 Texture 类中实现一个新方法 Vector3f getColorBilinear(float u, float v) 并 通过 fragment shader 调用它。为了使双线性插值的效果更加明显，你应该 考虑选择更小的纹理图。请同时提交纹理插值与双线性纹理插值的结果，并 进行比较。

    本题中，我刚开始做的时候发现svg格式无法被读取，后来我看了BBS论坛上，下载了别人已经压缩好的图片`spot_texture2.png`，之后在`Texture.hpp`中添加了一个`getColorBilinear`函数，刚开始的代码是下面这样的：

    ```c++
    Eigen::Vector3f getColorBilinear(float u, float v)
    {
      auto u_img = u * width;
      auto v_img = v * height;
    
      auto u00X = floor(u_img), u00Y = floor(v_img);
      auto u10X = ceil(u_img), u10Y = floor(v_img);
      auto u01X = floor(u_img), u01Y = ceil(v_img);
      auto u11X = ceil(u_img), u11Y = ceil(v_img);
    
      auto color00 = image_data.at<cv::Vec3b>(u00X, u00Y);
      auto color10 = image_data.at<cv::Vec3b>(u10X, u10Y);
      auto color01 = image_data.at<cv::Vec3b>(u01X, u01Y);
      auto color11 = image_data.at<cv::Vec3b>(u11X, u11Y);
    
      auto ColorDown = color00 + (u_img - u00X) * (color10 - color00);
      auto ColorUpon = color01 + (u_img - u01X) * (color11 - color01);
    
      auto color = ColorDown + (v_img - u00Y) * (ColorUpon - ColorDown);
    
      return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    ```

    结果不对，后来将`colorXX`获取的方式改为`getcolor`而不是`image_data`，再进行插值，发现解决了问题。可以正确显示双线性插值后的结果，对比了小牛的头部和腿部，效果还不错。





















