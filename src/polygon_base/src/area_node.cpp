#include <cstdio>
#include <pluginlib/class_loader.hpp>
#include <polygon_base/regular_polygon.hpp>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  // 关键！定义在 class_loader.hpp中（ROS2源码）
  // 1. 模板化 为 base class（polygon_base::RegularPolygon）
  // 2. 第一个参数是 base class的 package 名
  // 3. 第二个参数是完整的base class类型名称（polygon_base::RegularPolygon）
  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("polygon_base", "polygon_base::RegularPolygon");

  // 使用shared ptr实例化类
  try
  {
    // 调用 createSharedInstance 去创建这个实例。其中，需要传入完整的插件类名称
    std::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createSharedInstance("polygon_plugins::Triangle");
    triangle->initialize(10.0);

    std::shared_ptr<polygon_base::RegularPolygon> square = poly_loader.createSharedInstance("polygon_plugins::Square");
    square->initialize(10.0);

    printf("Triangle area: %.2f\n", triangle->area());
    printf("Square area: %.2f\n", square->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}
