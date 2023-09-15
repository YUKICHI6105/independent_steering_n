# independent_steering_n
独立ステアリング機構N。
NはNoTemplateのN。

CRSLibtmpにあるMechanismの中のコードのテンプレート部分を排した(それらテンプレートに型を入れて具体化した)もの。
C++知識が少なくても読める。
**あくまで手動制御用**。
軌道が既に分かっているなら、そこへの追従制御を行うべき。

# 特徴
カニ制御とくるくる制御、を実装。
どちらで制御するかは、切り替えるたびにTopicの`/<node_name>/control_mode`経由で切り替える。
カニ制御の指令値は`/<node_name>/vel`に`geometry_msgs/msg/`、くるくる制御の指令値は`/<node_name>/rot`で受け取る。


# 使い方
node.hppがノード本体である。
これを直接rclcpp::executors内のエグゼキュータに渡してもよし、RCLCPP_COMPONENTS_REGISTER_NODEに渡してもよし。

