#include <rclcpp/rclcpp.hpp>

#include <feetech_sts_interface/feetech_sts_interface.hpp>
#include <feetech_sts_example/read_pos.hpp>

int main(int argc, char * argv[])
{
  // ROS2の初期化処理
  rclcpp::init(argc, argv);
  // Executor(ノード実行器)のインスタンスを作成
  rclcpp::executors::MultiThreadedExecutor exec;
  // Publishノードの実体をインスタンス化し、その共有ポインタを取得
  const auto read_pos = std::make_shared<feetech_sts_interface::PubFeetechNode>(rclcpp::NodeOptions());
  // Executorに各ノードを登録する。
  // exec.add_node(std::make_shared<tutorial::Publish>(rclcpp::NodeOptions()));
  // とするとスッキリかけて良さそうに見えるが、add_nodeの処理が終わった瞬間に参照カウントが0になってノードの実体が破棄されてしまうのでやらないこと
  exec.add_node(read_pos);
  // spin（callbackの実行フラグとなっている各種イベントが実行されるまで待機する）処理
  exec.spin();
  // ROS2の終了処理
  rclcpp::shutdown();
}