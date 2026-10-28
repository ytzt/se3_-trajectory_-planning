# 理论基础
轨迹规划问题本质上在于寻找属于不同领域的两个元素之间的关系：时间和空间。因此，轨迹通常表示为时间的参数函数。
本仓库实现了轨迹规划的常见方法。

# 基本轨迹的解析表达式
## 多项式轨迹

n次多项式，以及k阶导数的表达式：
$$k!\,a_k + (k+1)!\,a_{k+1}t_j + \dots + \frac{n!}{(n-k)!}\,a_n t_j^{n-k} = q^{(k)}(t_j)$$
转化为矩阵形式：
$$
\mathbf{M}\mathbf{a} = \mathbf{b}
$$

### 线性轨迹
起点$q_0$到终点$q_1$在$[t_{0},t_{1}]$时间内的直线运动轨迹定义：
$$q(t) = a_{0} + a_{1}(t - t_{0}).$$
- 轨迹示例

  <p align="center"><img src="images/1st_trajectory.png" alt="线性轨迹示例" title="直线运动" width="500" /></p>

### 抛物线轨迹
起点$q_0$到终点$q_1$在$[t_{0},t_{1}]$时间内的抛物线轨迹定义：
$$q_{a}(t) = a_{0} + a_{1}(t - t_{0}) + a_{2}(t - t_{0})^{2}$$
- 轨迹示例

  <p align="center"><img src="images/2nd_trajectory.png" alt="抛物线轨迹示例" title="抛物线轨迹" width="500" /></p>

- 非对称恒加速度轨迹
$$% 自动编号可放在 align 环境里，这里用 aligned 供 cases 内部对齐
\left\{
\begin{alignedat}{4}
q_a(t_0) &= a_0 &&= q_0, \\[2pt]
\dot q_a(t_0) &= a_1 &&= v_0, \\[2pt]
q_b(t_1) &= a_3 + a_4(t_1-t_f) + a_5{(t_1-t_f)^2} &&= q_1, \\[2pt]
\dot q_b(t_1) &= a_4 + 2a_5(t_1-t_f) &&= v_1, \\[2pt]
q_a(t_f) &= a_0 + a_1(t_f-t_0) + a_2{(t_f-t_0)^2} &&= a_3 &&= q_b(t_f), \\[2pt]
\dot q_a(t_f) &= a_1 + 2a_2(t_f-t_0) &&= a_4 &&= \dot q_b(t_f).
\end{alignedat}
\right.$$

<p align="center"><img src="images/asym_constant_acc.png" alt="非对称恒加速度轨迹" title="非对称恒加速度" width="500" /></p>

### 三次轨迹
通过给定$[t_{0},t_{1}]$时刻位置和速度（分别为 $q_0, q_1$ 和 $v_0, v_1$），则可用三次多项式表达。
$$q_{a}(t) = a_{0} + a_{1}(t - t_{0}) + a_{2}(t - t_{0})^{2}+a_{3}(t - t_{0})^{3}$$
对应的解：
$$
\begin{cases}
a_{0} = q_{0}, \\[4pt]
a_{1} = v_{0}, \\[4pt]
a_{2} = \dfrac{3h - (2v_{0} + v_{1})T}{T^{2}}, \\[8pt]
a_{3} = \dfrac{-2h + (v_{0} + v_{1})T}{T^{3}}.
\end{cases}
$$

在定义一组过N点的轨迹时，如果中间点速度不总是指定，可以使用heuristic rules确定中间速度。
$$% 定义分段函数
v_k =
\begin{cases}
0 & \text{if }\ \mathrm{sign}(d_k) \neq \mathrm{sign}(d_{k+1}) \\[6pt]
\dfrac{1}{2}(d_k + d_{k+1}) & \text{if }\ \mathrm{sign}(d_k) = \mathrm{sign}(d_{k+1})
\end{cases}
\quad d_k = \frac{q_k - q_{k-1}}{t_k - t_{k-1}}$$

- 轨迹示例

  <p align="center"><img src="images/3rd_trajectory.png" alt="三次轨迹示例" title="三次轨迹" width="500" /></p>

### 五次轨迹
起点$q_0$到终点$q_1$在$[t_{0},t_{1}]$时间内的五次多项式轨迹定义：
$$q_{a}(t) = a_{0} + a_{1}(t - t_{0}) + a_{2}(t - t_{0})^{2}+a_{3}(t - t_{0})^{3}+a_{4}(t - t_{0})^{4}+a_{5}(t - t_{0})^{5}$$
对各个系数进行求解可得：
$$\begin{cases}
a_{0} = q_{0}, \\[4pt]
a_{1} = v_{0}, \\[4pt]
a_{2} = \dfrac{1}{2}a_{0}, \\[8pt]
a_{3} = \dfrac{1}{2T^{3}}\bigl[20h - (8v_{1} + 12v_{0})T - (3acc_{0} - acc_{1})T^{2}\bigr], \\[12pt]
a_{4} = \dfrac{1}{2T^{4}}\bigl[-30h + (14v_{1} + 16v_{0})T + (3acc_{0} - 2acc_{1})T^{2}\bigr], \\[12pt]
a_{5} = \dfrac{1}{2T^{5}}\bigl[12h - 6(v_{0} + v_{1})T + (acc_{0} - acc_{1})T^{2}\bigr].
\end{cases}$$

- 轨迹示例

  <p align="center"><img src="images/5th_trajectory.png" alt="五次轨迹示例" title="五次轨迹" width="500" /></p>

## 三角轨迹
### 谐波轨迹
其轨迹定义如下：
$$q(t) = \frac{h}{2}\left[1 - \cos\frac{\pi(t - t_0)}{T}\right] + q_0$$

$$s(\theta) = R(1 - \cos\theta)$$

<p align="center"><img src="images/harmonic_schematic.png" alt="谐波运动律示意图" title="s(θ)=R(1-cosθ)" width="500" /></p>

- 轨迹示例

  <p align="center"><img src="images/harmonic_trajectory.png" alt="谐波轨迹示例" title="谐波轨迹" width="500" /></p>


### 摆线轨迹
其轨迹定义如下：
$$% 变量定义
q(t) = (q_1 - q_0)\left[\frac{t - t_0}{\tau} - \frac{1}{2\pi}\sin\frac{2\pi(t - t_0)}{\tau}\right] + q_0
\qquad \text{其中}\ \tau = t_1 - t_0$$

<p align="center"><img src="images/cycloid_schematic.png" alt="摆线曲线示意图" title="摆线" width="500" /></p>

- 轨迹示例

  <p align="center"><img src="images/cycloidal_trajectory.png" alt="摆线轨迹示例" title="摆线轨迹" width="500" /></p>





# 基本轨迹组合

## 梯形轨迹
### 原理
应用场景，预设加速度以及速度（$a_a= a_{max}, v_v<=v_{max}$)
判断有无恒速阶段
$$\Delta = h a_{\max} - \frac{v_0^{2} + v_1^{2}}{2}.$$
Case1
$$\Delta > v_{\max}^{2}, v_{\rm v} = v_{\max}$$
$$T_a = \frac{v_{\max} - v_0}{a_{\max}}, \quad
  T_d = \frac{v_{\max} - v_1}{a_{\max}}, T = \frac{h}{v_{\max}} - \frac{v_0+v_{\max}}{2v_{\max}}T_a- \frac{v_1+v_{\max}}{2v_{\max}}T_d
 .$$
CASE2 
$$v_{\rm v} = v_{\lim} = \sqrt{h a_{\max} + \frac{v_0^{2} + v_1^{2}}{2}} < v_{\max}$$
$$T_a = \frac{v_{\lim} - v_0}{a_{\max}}, \quad
  T_d = \frac{v_{\lim} - v_1}{a_{\max}}, \quad
  T = T_a + T_d.$$

- 加速阶段
$$
\begin{aligned}
q(t)       &= q_0 + v_0(t - t_0) + \frac{v_{\rm v} - v_0}{2T_a}(t - t_0)^2, \\[4pt]
\dot q(t)  &= v_0 + \frac{v_{\rm v} - v_0}{T_a}(t - t_0), \\[4pt]
\ddot q(t) &= \frac{v_{\rm v} - v_0}{T_a} = a_{\rm a}.
\end{aligned}
$$
- 恒速阶段
$$
\begin{aligned}
q(t)       &= q_0 + v_0 T_a + \frac{1}{2}a_{\rm a}T_a^{2}
              + v_{\rm v}\!\left(t - t_0 - \frac{T_a}{2}\right), \\[4pt]
\dot q(t)  &= v_{\rm v}, \\[4pt]
\ddot q(t) &= 0.
\end{aligned}
$$
- 减速阶段
$$\begin{aligned}
q(t)       &= q_1 - v_1(t_1 - t) - \frac{v_{\rm v} - v_1}{2T_d}(t_1 - t)^2, \\[4pt]
\dot q(t)  &= v_1 + \frac{v_{\rm v} - v_1}{T_d}(t_1 - t), \\[4pt]
\ddot q(t) &= -\frac{v_{\rm v} - v_1}{T_d} = -a_{\rm a}.
\end{aligned}$$

### 代码实现
```cpp
// 创建规划器（最大加速度 20，最大速度 15，采样周期 10ms）
TrapezoidalPlanner planner(20, 15, 0.01);

// 单段轨迹：从 q0=0 到 q1=10，起止速度均为 0
auto [time, pos, vel, acc] = planner.plan(0.0, 10.0, 0.0, 0.0);
```

### 途径多点的梯形轨迹实现
通过指定中间点速度，可以避免在中间点速度频繁为0.
```cpp
// 创建规划器（最大加速度 20，最大速度 15，采样周期 10ms）
TrapezoidalPlanner planner(20, 15, 0.01);

// 定义路径点
std::vector<double> waypoints = {0.0, 20, 50, 40, 20};

// 生成多段轨迹
auto [time, pos, vel, acc] = planner.planMultiSegment(waypoints);
```

<p align="center"><img src="images/trapezoidal_multipoint.png" alt="途径多点的梯形轨迹" title="多点梯形轨迹" width="500" /></p>



## 双S形轨迹
又叫“七段式”轨迹，钟形轨迹，条件，对最大速度，加速度，加加速度（各自互为相反数）进行约束。可弥补梯形轨迹加速度不连续的问题
### 原理
- 加速阶段
$$t\in[0,\,T_{j1}]$$
$$
\begin{aligned}
q(t)       &= q_0 + v_0 t + \frac{j_{\max}}{6}t^{3},\\[4pt]
\dot q(t)  &= v_0 + \frac{j_{\max}}{2}t^{2},\\[4pt]
\ddot q(t) &= j_{\max}t,\\[4pt]
q^{(3)}(t) &= j_{\max}.
\end{aligned}
$$

$$t\in[T_{j1},\,T_a-T_{j1}]$$
$$
\begin{aligned}
q(t)       &= q_0 + v_0 t + \frac{a_{\lim}}{6T_{j1}}
              \bigl(3t^{2}-3T_{j1}t+T_{j1}^{2}\bigr),\\[4pt]
\dot q(t)  &= v_0 + a_{\lim}\bigl(t-\tfrac{T_{j1}}{2}\bigr),\\[4pt]
\ddot q(t) &= a_{\lim}=\frac{j_{\max}}{T_{j1}},\\[4pt]
q^{(3)}(t) &= 0.
\end{aligned}$$
$$t\in[T_a-T_{j1},\,T_a]$$
$$
\begin{aligned}
q(t)       &= q_0 + (v_{\lim}+v_0)\frac{T_a}{2}
              -v_{\lim}(T_a-t)-\frac{j_{\min}}{6}(T_a-t)^{3},\\[4pt]
\dot q(t)  &= v_{\lim} + \frac{j_{\min}}{2}(T_a-t)^{2},\\[4pt]
\ddot q(t) &= -j_{\min}(T_a-t),\\[4pt]
q^{(3)}(t) &= j_{\min}=-j_{\max}.
\end{aligned}
$$
- 恒速阶段
$$t\in[T_a,\,T_a+T_v]$$
$$
\begin{aligned}
q(t)       &= q_0 + (v_{\lim}+v_0)\frac{T_a}{2}
              +v_{\lim}(t-T_a),\\[4pt]
\dot q(t)  &= v_{\lim},\\[4pt]
\ddot q(t) &= 0,\\[4pt]
q^{(3)}(t) &= 0.
\end{aligned}
$$

- 减速阶段
$$t\in[T-T_d,\,T-T_d+T_{j2}]$$
$$
\begin{aligned}
q(t)       &= q_1 - (v_{\lim}+v_1)\frac{T_d}{2}
              +v_{\lim}(t-T+T_d)-\frac{j_{\max}}{6}(t-T+T_d)^{3},\\[4pt]
\dot q(t)  &= v_{\lim} - \frac{j_{\max}}{2}(t-T+T_d)^{2},\\[4pt]
\ddot q(t) &= -j_{\max}(t-T+T_d),\\[4pt]
q^{(3)}(t) &= j_{\min}=-j_{\max}.
\end{aligned}
$$

$$ t\in[T-T_d+T_{j2},\,T-T_{j2}]$$
$$
\begin{aligned}
q(t)       &= q_1 - (v_{\lim}+v_1)\frac{T_d}{2}
              +v_{\lim}(t-T+T_d)\\[4pt]
           &\quad +\frac{a_{\lim d}}{6T_{j2}}
             \Bigl[3(t-T+T_d)^{2}-3T_{j2}(t-T+T_d)+T_{j2}^{2}\Bigr],\\[4pt]
\dot q(t)  &= v_{\lim} + a_{\lim d}\bigl(t-T+T_d-\tfrac{T_{j2}}{2}\bigr),\\[4pt]
\ddot q(t) &= a_{\lim d}=\frac{-j_{\max}}{T_{j2}},\\[4pt]
q^{(3)}(t) &= 0.
\end{aligned}
$$
$$t\in[T-T_{j2},\,T]$$
$$
\begin{aligned}
q(t)       &= q_1 - v_1(T-t)-\frac{j_{\max}}{6}(T-t)^{3},\\[4pt]
\dot q(t)  &= v_1 + \frac{j_{\max}}{2}(T-t)^{2},\\[4pt]
\ddot q(t) &= -j_{\max}(T-t),\\[4pt]
q^{(3)}(t) &= j_{\max}.
\end{aligned}
$$

### 代码实现

<p align="center"><img src="images/double_s_trajectory.png" alt="双S形轨迹示例" title="双S轨迹" width="500" /></p>

```cpp
// 约束参数：最大速度 10，最大加速度 20，最大加加速度 60
DoubleSTrajectoryPlanner planner(10.0, 20.0, 60.0);

// 生成轨迹：采样 1ms，位置 30 → 0，起始速度 0，结束速度 1
auto [time, positions, velocities, accelerations, jerk] =
    planner.plan(0.001, 30.0, 0.0, 0.0, 1.0);
```


# 多点轨迹
## 三次样条曲线
当给定n+1个点时，可以使用n个p阶多项式代替单个n阶插值多项式，每个多项式代表一段轨迹，这种方式定义的整体函数s(t)称为p次样条。样条函数由以下形式给出：
$$
\begin{aligned}
s(t) &= \{q_k(t),\; t\in[t_k,t_{k+1}],\; k=0,\dots,n-1\} \\
q_k(t) &= a_{k0} + a_{k1}(t - t_k) + a_{k2}(t - t_k)^2 + a_{k3}(t - t_k)^3
\end{aligned}
$$

## 多点，指定初始速度，最终速度
```cpp
vector<double> times = {0, 5, 7, 8, 10, 15, 18};
vector<double> positions = {3, -2, -5, 0, 6, 12, 8};
double v0 = 2.0;   // 初始速度
double vn = -3.0;  // 最终速度
```

<p align="center"><img src="images/cubic_spline_v0vn.png" alt="三次样条轨迹 v0=2, vn=-3" title="样条轨迹" width="500" /></p>

## 指定初始速度、最终速度，初始加速度、最终加速度。
样条曲线是二阶导数连续的函数，但通常无法同时指定初始速度、最终速度和加速度。因此，样条曲线在其端点处会出现速度或加速度的不连续性。如果这些不连续性造成问题，可以采用以下方法：
1. 可以使用 5 次多项式函数来表示首尾曲线段，但缺点是这些曲线段的超调量较大，计算量略有增加。
2. 在首尾曲线段分别添加两个额外的自由点，并通过设定所需的速度和加速度的初始和最终值来计算这两个自由点的值。

```cpp
vector<double> times = {0, 5, 7, 8, 10, 15, 18};
        vector<double> positions = {3, -2, -5, 0, 6, 12, 8};
        double v0 = 0.0;   // 初始速度
        double vn = 0.0;  // 最终速度
        double a0 = 0.0;   // 初始加速度
        double an = 0.0;   // 最终加速度
```



<p align="center"><img src="images/cubic_spline_trajectory.png" alt="三次样条轨迹示例" title="三次样条轨迹" width="500" /></p>

# 几何路径规划
## 全局插值
B样条函数可以解决更高阶导数的连续性（比如加加速度的连续性），特别的，3次B样条曲线，就是标准的三次样条曲线。使用B样条表示，可以利用B样条函数本身的性质来求解出所需要的不同次数的曲线，更具有通用性。
### B样条基函数
#### 数学原理
- 设节点向量  
$$u = [u_0, u_1, \dots, u_{n_{\text{knot}}}] \quad \text{且} \quad u_j \le u_{j+1}  $$

- 0 阶（分段常数）基函数
$$B_j^0(u) =  
\begin{cases}  
1, & u_j \le u < u_{j+1} \\[4pt]  
0, & \text{otherwise}  
\end{cases} $$
- 高阶递归定义
$$B_j^p(u) =  
\frac{u - u_j}{u_{j+p} - u_j} B_j^{p-1}(u) +  
\frac{u_{j+p+1} - u}{u_{j+p+1} - u_{j+1}} B_{j+1}^{p-1}(u),  
\quad p > 0 $$
p阶B样条基函数只在以下区间内有数值。
$$B_j^p(u) = 0 \quad \text{for} \quad u \notin [u_j, u_{j+p+1})$$

#### 不同次数的B样条基函数展示
节点：`const std::vector<double> knotsNormal{0.0, 1.0, 2.0, 4.0, 7.0};`
- 一阶B样条基函数

  <p align="center"><img src="images/bspline_basis_1st.png" alt="一阶B样条基函数" title="一阶B样条基函数" width="500" /></p>

- 二阶B样条基函数

  <p align="center"><img src="images/bspline_basis_2nd.png" alt="二阶B样条基函数" title="二阶B样条基函数" width="500" /></p>

- 三阶B样条基函数

  <p align="center"><img src="images/bspline_basis_3rd.png" alt="三阶B样条基函数" title="三阶B样条基函数" width="500" /></p>

### 基于B样条插值一组点
对一组点 qₖ (k = 0,…,n) 进行插值的最简单方法是使用 p 次 B 样条曲线 s(u)。
第一步是为每个点 qₖ 选择参数 ūₖ（可视为“时间瞬间”），并构造合适的节点向量 u = [u₀,…,uₙₖₙₒₜ]，其中 nₖₙₒₜ 为节点数。然后便可建立 m+1 个线性方程组，未知数是控制点 pⱼ，通过要求曲线在 ūₖ 处穿过每个点 qₖ 得到：
$$\mathbf{q}_k = s(\bar{u}_k) = \sum_{j=0}^{m} \mathbf{p}_j B_j^p(\bar{u}_k)$$
$$\mathbf{q}_k^T =
\begin{bmatrix}
B_0^p(\bar{u}_k) & B_1^p(\bar{u}_k) & \cdots & B_m^p(\bar{u}_k)
\end{bmatrix}
\begin{bmatrix}
\mathbf{p}_0^T \\ \mathbf{p}_1^T \\ \vdots \\ \mathbf{p}_m^T
\end{bmatrix}$$
#### 节点参数定义
每个点 qₖ 对应的参数$\bar u_k$限制在[0,1]范围中：
$$\bar u_0 = 0, \quad \bar u_n = 1$$
$\bar u_k $ 最常见的选择为：
- 等距分布
$$\bar u_k = \frac{k}{n}.$$

- 弦长分布
$$
\bar u_k = \bar u_{k-1} + \frac{|q_k - q_{k-1}|}{d}, \quad \text{其中}\quad d = \sum_{i=1}^{n} |q_i - q_{i-1}|.  
$$

- 向心分布

$$
\bar u_k = \bar u_{k-1} + \frac{|q_k - q_{k-1}|^{\mu}}{d}, \quad \text{其中}\quad d = \sum_{i=1}^{n} |q_i - q_{i-1}|^{\mu}.  
$$

#### 控制点的求解
节点总数为$n_{knot}+1 = n +1 + 2p$,其中，n+1为途径点个数，p为B样条阶数。
控制点的数量（$m+1$）与节点数量($n_{knot}+1$)之间的关系为 $n_{knot}=m+p+1$
对于三阶B样条，要确定n+3个控制点，除了n+1个途径点条件外，还需要引入两个约束条件，可以选择端点处的一阶导数（$t_0$,$t_n$)
- 节点向量的确定
$$u_0 = u_1 = u_2 = \bar u_0, \quad
u_{j+3} = \bar u_j \ (j = 0, \dots, n), \quad
u_{n+4} = u_{n+5} = u_{n+6} = \bar u_n
$$
- 边界条件

$$\begin{aligned}
p_0 &= q_0, \\
\frac{u_4 - u_3}{3} t_0 &= p_1 - p_0 \quad \Longrightarrow \quad
p_1 = p_0 + \frac{u_4}{3} t_0, \\
\frac{1 - u_{n+3}}{3} t_n &= p_{n+2} - p_{n+1} \quad \Longrightarrow \quad
p_{n+1} = p_{n+2} - \frac{1 - u_{n+2}}{3} t_n, \\
p_{n+2} &= q_n.
\end{aligned}
$$
- 中间点求解
$$q_k = B_k^3(\bar u_k) p_k + B_{k+1}^3(\bar u_k) p_{k+1} + B_{k+2}^3(\bar u_k) p_{k+2}.$$
$$\mathbf{B} \mathbf{P} = \mathbf{R},$$
$$\mathbf{B} =
\begin{bmatrix}
B_2^3(\bar u_1) & B_3^3(\bar u_1) & 0 & \cdots & 0 \\
B_2^3(\bar u_2) & B_3^3(\bar u_2) & B_4^3(\bar u_2) & \ddots & \vdots \\
0 & B_3^3(\bar u_3) & B_4^3(\bar u_3) & \ddots & 0 \\
\vdots & \ddots & \ddots & \ddots & B_n^3(\bar u_{n-1}) \\
0 & \cdots & 0 & B_{n-1}^3(\bar u_{n-1}) & B_n^3(\bar u_{n-1})
\end{bmatrix}$$

$$\mathbf{P} = [p_2^T, p_3^T, \dots, p_n^T]^T$$
$$\mathbf{R} =
\begin{bmatrix}
q_1^T - B_1^3(\bar u_1) p_1^T \\
q_2^T \\
\vdots \\
q_{n-1}^T - B_{n+1}^3(\bar u_{n-1}) p_{n+1}^T
\end{bmatrix}$$
通过托马斯算法可以对控制点P进行求解。


#### 代码实现
通过3阶B样条确定一条路径：

```cpp
std::vector<Vector3d> points = {
        {83, -54, 119}, {-64, 10, 124}, {42, 79, 226}, 
        {-98, 23, 222}, {-13, 125, 102}, {140, 81, 92},
        {43, 32, 92}, {-65, -17, 134}, {-45, -89, 182}, {71, 90, 192}
    };
```

<p align="center"><img src="images/bspline_visualization.png" alt="三阶B样条插值路径" title="B样条路径" width="500" /></p>





# 从几何路径到轨迹
- 参数化几何路径及其运动率可定义为：
$$\tilde{\boldsymbol{p}}(t) = \boldsymbol{p}\bigl(u(t)\bigr)$$
- 其导数可通过链式法计算：
$$\dot{\tilde{\boldsymbol{p}}}(t) = \frac{\mathrm{d}\boldsymbol{p}}{\mathrm{d}u}\,\dot{u}(t)$$
$$\ddot{\tilde{\boldsymbol{p}}}(t) = \frac{\mathrm{d}^{2}\boldsymbol{p}}{\mathrm{d}u^{2}}\,\dot{u}^{2}(t) + \frac{\mathrm{d}\boldsymbol{p}}{\mathrm{d}u}\,\ddot{u}(t)$$


## 定值缩放
- 恒定比例缩放（u = λt）
$$\tilde{\boldsymbol{p}}^{(1)}(t) = \lambda\,\frac{\mathrm{d}\boldsymbol{p}}{\mathrm{d}u}$$
$$\tilde{\boldsymbol{p}}^{(2)}(t) = \lambda^{2}\,\frac{\mathrm{d}^{2}\boldsymbol{p}}{\mathrm{d}u^{2}}$$
$$\tilde{\boldsymbol{p}}^{(3)}(t) = \lambda^{3}\,\frac{\mathrm{d}^{3}\boldsymbol{p}}{\mathrm{d}u^{3}}$$
在期望最大速度、加速度、加加速度的约束下的缩放系数计算：
$$\lambda = \min\left\{
\frac{v_{\max}}{\|\boldsymbol{p}^{(1)}(u)\|_{\max}},\;
\frac{a_{\max}}{\|\boldsymbol{p}^{(2)}(u)\|_{\max}},\;
\frac{j_{\max}}{\|\boldsymbol{p}^{(3)}(u)\|_{\max}},\;\dots
\right\}$$


- u=t的情况

  <p align="center"><img src="images/Direct_Parametrization_analysis.png" alt="u=t 直接参数化" title="u=t" width="500" /></p>

- 设置最大V约束的情况（V_MAX = 10, u=0.37t)

  <p align="center"><img src="images/Scaled_Parametrization_analysis.png" alt="定值缩放 V_MAX=10" title="定值缩放" width="500" /></p>

## 恒定速度运动率
设定这样的u(t)，满足轨迹的速度是一个常数：
$$|\dot{\tilde{\boldsymbol{p}}}(t)| = v_c \quad (\text{常数})$$
$u(t_k)$可以按照泰勒进行离散展开：
$$u_{k+1} = u_k + T_s \dot{u}_k + \frac{T_s^2}{2} \ddot{u}_k + O\!\left(\frac{T_s^n}{n!} u^{(n)}_k\right), \quad k = 0, 1, \dots$$
根据参数化轨迹的链式求到关系，可以求出u(t)的导数
$$\dot{u}(t) = \frac{v_c}{\left\|\dfrac{\mathrm{d}\boldsymbol{p}}{\mathrm{d}u}\right\|}$$
$$\ddot{u}(t) = -v_c^2 \frac{\dfrac{\mathrm{d}\boldsymbol{p}}{\mathrm{d}u}^\top \dfrac{\mathrm{d}^2\boldsymbol{p}}{\mathrm{d}u^2}}{\left\|\dfrac{\mathrm{d}\boldsymbol{p}}{\mathrm{d}u}\right\|^4}$$

进而得到满足期望运动速度轨迹的运动率。

### 代码实现
```cpp
//定义途径点
 std::vector<Eigen::Vector2d> points = {
        {0, 0}, {1, 2},{2, 1}, {4, 3}, {5, 2}, {6, 3}, 
        {7, 0}, {8, -1}, {9, -1}, {10, 2}
    };

// 3. 创建轨迹生成器
    PathToTrajectory<2> trajectory(path_func, deriv_func);
    
    // 4. 设置运动参数
    trajectory.setMotionParameters(10.0, 0.001); // v_max=10, Ts=0.01s
   // 一阶泰勒近似
    trajectory.generateFirstOrderTaylor(time_points, u_points, positions, velocities);
    saveTrajectoryData<2>("first_order_taylor.csv", time_points, u_points, positions, velocities);
```
- 一阶泰勒展开
速度变化9.85-10.12

<p align="center"><img src="images/First_Order_Taylor_analysis.png" alt="一阶泰勒展开" title="一阶泰勒展开" width="500" /></p>


- 二阶泰勒展开
速度变化9.99-10

<p align="center"><img src="images/Second_Order_Taylor_analysis.png" alt="二阶泰勒展开" title="二阶泰勒展开" width="500" /></p>

## 钟形速度运动率
- 一般情况下，轨迹的导数不是常量，而是关于时间的函数。
$$|\dot{\tilde{\boldsymbol{p}}}(t)| = v(t), \qquad
|\ddot{\tilde{\boldsymbol{p}}}(t)| = a(t)$$

- 这种情况下 $u(t_k)$ 得到的泰勒展开为：
$$u_{k+1} = u_{k} + \frac{v_{k}T_{s}}{\;\left\|\dfrac{\mathrm{d}\boldsymbol{p}}{\mathrm{d}u}\right\|_{u_k}} + \frac{T_{s}^{2}}{2}\left[\frac{a_{k}}{\;\left\|\dfrac{\mathrm{d}\boldsymbol{p}}{\mathrm{d}u}\right\|_{u_k}} - \frac{v_{k}^{2}\; \dfrac{\mathrm{d}\boldsymbol{p}}{\mathrm{d}u}^{\top}\dfrac{\mathrm{d}^{2}\boldsymbol{p}}{\mathrm{d}u^{2}}}{\;\left\|\dfrac{\mathrm{d}\boldsymbol{p}}{\mathrm{d}u}\right\|^{4}_{u_k}}\right]$$
- 推导过程
两边点乘切向量
$$\left(\frac{\mathrm{d}\boldsymbol{p}}{\mathrm{d}u} \cdot \frac{\mathrm{d}\boldsymbol{p}}{\mathrm{d}u}\right)\ddot{u} = \frac{\mathrm{d}\boldsymbol{p}}{\mathrm{d}u} \cdot \left(\boldsymbol{a} - \frac{\mathrm{d}^{2}\boldsymbol{p}}{\mathrm{d}u^{2}}\,\dot{u}^{2}\right)$$
$$\ddot{u} = \frac{1}{\left\|\dfrac{\mathrm{d}\boldsymbol{p}}{\mathrm{d}u}\right\|^{2}}\left[\dfrac{\mathrm{d}\boldsymbol{p}}{\mathrm{d}u} \cdot \boldsymbol{a}
- \left(
\dfrac{\mathrm{d}\boldsymbol{p}}{\mathrm{d}u} \cdot \dfrac{\mathrm{d}^{2}\boldsymbol{p}}{\mathrm{d}u^{2}}\right)\dot{u}^{2}\right]$$

- 通过几何路径对U积分得到路径的几何长度，然后根据双S曲线规划得到期望速度曲线V（t)，对U(t)进行泰勒展开计算期望的运动率，进而计算期望的轨迹。

执行流程如下：


```mermaid
flowchart LR
A([开始]) --> B[1. 通过离散点生成几何路径]
    B --> C[2. 规划期望速度/加速度]
    C --> D[3. 计算运动率]
    D --> E[4. 生成轨迹或高阶律]
    E --> F([结束])
```

### 代码实现
```cpp
auto [Stime, Su_points, Spositions,  Svelocities] = 
    trajectory.generateTrajectoryWithVariableFeedrate(
        0.001,  // 采样时间 10ms
        10.0,   // 最大速度 m/s
        50.0,   // 最大加速度 m/s²
        100.0,   // 最大加加速度 m/s³
        0.0,   // 起始速度
        0.0    // 结束速度
    );
```

<p align="center"><img src="images/S_Motion_Law_analysis.png" alt="钟形速度运动率轨迹" title="S形运动率" width="500" /></p>







# 参考文献
Biagiotti, Luigi, Melchiorri, Claudio. Trajectory Planning for Automatic Machines and Robots [J]. 10.1007/978-3-540-85629-0.