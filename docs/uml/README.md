# FCR ROS2 UML Diagrams

PlantUML (.puml) 格式的完整UML图集，覆盖系统架构各层面。

## 图表索引

| # | 文件 | 类型 | 内容 |
|---|------|------|------|
| 1 | `01_package_diagram.puml` | **Package Diagram** | 7个包的依赖关系、内部组件、编译/运行时依赖 |
| 2 | `02_class_diagram.puml` | **Class Diagram** | 控制器继承体系、硬件接口、感知流水线、工具类、消息定义 |
| 3 | `03_sequence_ibvs.puml` | **Sequence Diagram** | IBVS视觉伺服完整时序：初始化→目标设置→50Hz控制循环→收敛 |
| 4 | `04_component_diagram.puml` | **Component Diagram** | ROS2 Node & Topic连接图、QoS标注、Callback Group分区 |
| 5 | `05_activity_servo_loop.puml` | **Activity Diagram** | 50Hz伺服控制循环活动图，含雅可比奇异检测和降级策略 |
| 6 | `06_deployment_diagram.puml` | **Deployment Diagram** | 工控机+Jetson+地面站三节点部署，DDS网络配置 |
| 7 | `07_state_machine.puml` | **State Machine** | VisualServo Action状态机：IDLE→CONVERGING→TRACKING→LOST→ERROR |
| 8 | `08_launch_sequence.puml` | **Sequence Diagram** | Launch分层启动时序：Phase1(0s)→Phase2(2s)→Phase3(3s)→Phase4(4s) |

## 渲染方式

### 方法 1: VS Code 插件
```bash
# 安装 PlantUML 插件
code --install-extension jebbs.plantuml
# 打开 .puml 文件 → Alt+D 预览
```

### 方法 2: 命令行
```bash
# 安装 plantuml (需要 Java)
sudo apt install plantuml

# 批量生成 PNG
cd docs/uml
for f in *.puml; do
  plantuml -tpng "$f"
done

# 生成 SVG (矢量图，推荐论文用)
for f in *.puml; do
  plantuml -tsvg "$f"
done
```

### 方法 3: 在线渲染
将 .puml 内容粘贴到 https://www.plantuml.com/plantuml/

### 方法 4: Python 库
```bash
pip install plantuml
python -c "from plantuml import PlantUML; PlantUML('http://www.plantuml.com/plantuml/').processes_file('01_package_diagram.puml')"
```
