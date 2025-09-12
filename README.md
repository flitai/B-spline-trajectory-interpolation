

### 算法输入

算法输入主要包括三个部分：

1.  **时间步长 (`ts`)**:
    * **类型**: `double`
    * **作用**: 这是一个标量值，用于定义均匀 B 样条的节点间距，直接影响轨迹的时间尺度和导数（速度、加速度）的大小。

2.  **航路点集合 (`point_set`)**:
    * **类型**: `std::vector<Eigen::Vector3d>`
    * **作用**: 这是一个三维向量的列表，包含了轨迹必须**穿越**的所有中间点。

3.  **端点导数约束 (`start_end_derivative`)**:
    * **类型**: `std::vector<Eigen::Vector3d>`
    * **作用**: 这是一个包含**四个**三维向量的列表，用于精确定义轨迹在起点和终点的状态。它必须严格按照以下顺序提供：
        1.  **起点速度** (v₀)
        2.  **终点速度** (vₘ)
        3.  **起点加速度** (a₀)
        4.  **终点加速度** (aₘ)



### 算法输出

`NonUniformBspline::parameterizeToBspline` 函数通过其最后一个参数**（通过引用传递）**来返回结果。

这个函数的输出是：

- **控制点矩阵 (`ctrl_pts`)**:
  - **类型**: `Eigen::MatrixXd&`
  - **作用**: 这是算法计算出的核心结果。它是一个二维矩阵，完整地定义了生成的 B 样条曲线。
  - **结构**:
    - 矩阵的**每一行**代表一个控制点。
    - 矩阵的**每一列**代表一个坐标维度（例如，第0列是 x，第1列是 y，第2列是 z）。
  - **大小**: 如果输入了 `K` 个航路点，那么输出的控制点数量将是 `K + 2` 个。因此，这个矩阵的大小将是 `(K + 2) x 3`。

### 代码组成

- ** C++ 主程序 (`main.cpp`)**: 用于调用您提供的 B 样条库，生成一个具体的轨迹案例，并将结果（航路点、控制点、采样轨迹）输出到 CSV 文件中。

- **Python 可视化脚本 (`visualize.py`)**: 用于读取 C++ 程序生成的 CSV 文件，并使用 Matplotlib 库绘制出与网页版风格类似的图表，包括轨迹图、速度图和加速度图。

- **`CMakeLists.txt` 文件**: 方便您轻松编译和链接 C++ 程序（需要 Eigen 库）。



### 如何运行

请按照以下步骤在本地计算机上编译和运行此测试套件。

**先决条件:**

- **C++ 编译器**: 如 `g++` 或 `clang++`。
- **CMake**: 版本 3.10 或更高。
- **Eigen 库**: 一个用于线性代数的 C++ 模板库。您可以使用包管理器进行安装（例如 `sudo apt-get install libeigen3-dev` on Ubuntu, `brew install eigen` on macOS）。
- **Python 3**: 以及 `pandas`、`numpy`、`matplotlib` 库。

**步骤 1: 设置项目结构**

将 `non_uniform_bspline.h`, `non_uniform_bspline.cpp` 以及 `main.cpp`, `visualize.py`, `CMakeLists.txt` 五个文件放在同一个目录下。

.
├── CMakeLists.txt
├── main.cpp
├── non_uniform_bspline.cpp
├── non_uniform_bspline.h
└── visualize.py

**步骤 2: 编译 C++ 程序**

打开终端，进入您的项目目录，然后执行以下命令：

```bash
mkdir build
cd build
cmake ..
make

如果一切顺利，`build` 目录下会生成一个名为 `bspline_test` 的可执行文件。

**步骤 3: 运行 C++ 程序生成数据**

在 `build` 目录下，运行程序：

```bash
./bspline_test

程序会输出 `waypoints.csv`, `control_points.csv`, 和 `trajectory.csv` 到您的项目**根目录**（上一级目录）。

**步骤 4: 安装 Python 依赖**

如果您尚未安装所需库，请在终端中运行：

```bash
pip install pandas numpy matplotlib seaborn

**步骤 5: 运行 Python 脚本进行可视化**

回到项目根目录，运行 Python 脚本：

```bash
python visualize.py

这会弹出一个包含三个图表的窗口，同时也会在项目根目录下保存一个名为 `trajectory_visualization.png` 的图像文件。
```

