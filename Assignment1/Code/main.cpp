#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;
inline float angle2radian(const float& angle) { return MY_PI / 180.0f * angle; }

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f translate;
  translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1, -eye_pos[2],
      0, 0, 0, 1;

  view = translate * view;

  return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle) {
  Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

  // TODO: Implement this function
  // Create the model matrix for rotating the triangle around the Z axis.
  // Then return it.
  rotation_angle = angle2radian(rotation_angle);

  model << cos(rotation_angle), -1.0 * sin(rotation_angle), 0, 0,
      sin(rotation_angle), cos(rotation_angle), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar) {
  // Students will implement this function

  Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

  // TODO: Implement this function
  // Create the projection matrix for the given parameters.
  // Then return it.
  // 该函数的目标是，将三角形的三个点 投影到 (x,y,z) 的正则化空间[-1,1]^3中
  // 构建：正交投影 M_ortho
  eye_fov = angle2radian(eye_fov);
  Eigen::Matrix4f M_ortho = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f M_ortho_translate = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f M_ortho_scale = Eigen::Matrix4f::Identity();
  auto t = zNear * tan(eye_fov / 2.0f);
  auto b = -t;
  auto n = zNear;
  auto f = zFar;
  auto l = b * aspect_ratio;
  auto r = t * aspect_ratio;
  M_ortho_translate << 1, 0, 0, -(r + l) / 2.0, 0, 1, 0, -(t + b) / 2.0, 0, 0,
      1, -(n + f) / 2.0, 0, 0, 0, 1;
  M_ortho_scale << 2.0 / (r - l), 0, 0, 0, 0, 2.0 / (t - b), 0, 0, 0, 0,
      2 / (n - f), 0, 0, 0, 0, 1;

  M_ortho = M_ortho_scale * M_ortho_translate;

  Eigen::Matrix4f M_persp_ortho = Eigen::Matrix4f::Identity();
  // 构建：透视-正交投影 M_persp_ortho
  n = zNear;
  f = zFar;
  M_persp_ortho << n, 0, 0, 0, 0, n, 0, 0, 0, 0, n + f, -n * f, 0, 0, 1, 0;

  projection = M_ortho * M_persp_ortho;

  return projection;
}

int main(int argc, const char** argv) {
  float angle = 0;
  bool command_line = false;
  std::string filename = "output.png";

  if (argc >= 3) {
    command_line = true;
    angle = std::stof(argv[2]);  // -r by default
    if (argc == 4) {
      filename = std::string(argv[3]);
    } else
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

    r.set_model(get_model_matrix(angle));
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
    } else if (key == 'd') {
      angle -= 10;
    }
  }

  return 0;
}
