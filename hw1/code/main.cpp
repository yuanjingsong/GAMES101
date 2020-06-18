#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

inline float abs_f(float a) {
    return a > 0 ? a : -a;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                0, 1, 0, -eye_pos[1],
                0, 0, 1,-eye_pos[2],
                0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    float angle = rotation_angle * MY_PI / 180.0f;
    model(0, 0) = cos(angle);
    model(0, 1) = -sin(angle);
    model(1, 0) = sin(angle);
    model(1, 1) = cos(angle);

    return model;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) 
{
    float length = sqrt(axis.x()*axis.x() + axis.y()*axis.y() + axis.z()*axis.z());

    float x_angle = std::acos(axis.x()/length);
    float y_angle = std::acos(axis.y()/length);
    float z_angle = std::acos(axis.z()/length);

    Eigen::Matrix4f xMat, yMat, zMat;
    xMat << 1, 0, 0, 0,
            0, cos(x_angle), -sin(x_angle), 0,
            0, sin(x_angle), cos(x_angle), 0,
            0, 0, 0, 1;

    yMat << cos(y_angle), 0, sin(y_angle), 0,
            0, 1, 0, 0,
            -sin(y_angle), 0, cos(y_angle), 0,
            0, 0, 0, 1;

    zMat << cos(z_angle), -sin(z_angle), 0, 0,
            sin(z_angle), cos(z_angle), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;


    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    view = xMat * yMat * zMat * view;
    return view;
}


Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    float eye_fov_angle = eye_fov * MY_PI / 180.0f;
    float r, l, t, b, n, f;
    t = tan(eye_fov_angle / 2.0) * abs_f(zNear);
    r = aspect_ratio * t;
    l = -r;
    b = -t; 
    n = zNear;
    f = zFar;
    Eigen::Matrix4f m1, m2;
    Eigen::Matrix4f PersToOrth;
    PersToOrth << zNear, 0, 0, 0,
                  0, zNear, 0, 0, 
                  0, 0, zNear + zFar, -zNear*zFar,
                  0, 0, 1, 0;
    
    m1 << 2/(r-l), 0, 0, 0,
            0, 2/(t-b), 0, 0,
            0, 0, 2/(n-f), 0,
            0, 0, 0, 1;

    m2 << 1, 0, 0, -(r + l)/2,
        0, 1, 0, -(t+b)/2,
        0, 0,  1, -(n+f)/2,
        0, 0, 0, 1;

    projection = m1 * m2 * PersToOrth * projection;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        Vector3f x = {0, 1, 0};
        r.set_model(get_rotation(x, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
