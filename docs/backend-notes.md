# 后端可用性：为什么走 ROS 2 而不是 unitree_sdk2py

本机是 Ubuntu 26.04 + Python **3.14**，这条路上有两个真实的坑，记录如下。

## 1. `cyclonedds` 的 Python 扩展在 Python 3.14 上编不出来（已解决）

编译 `clayer/pysertype.c` 时报：

    error: implicit declaration of function '_Py_IsFinalizing'; did you mean 'Py_IsFinalizing'?

`_Py_IsFinalizing` 是私有 API，3.13 起被公开的 `Py_IsFinalizing` 取代，3.14 里彻底删掉。
一行补丁即可：

    sed -i 's/_Py_IsFinalizing()/Py_IsFinalizing()/g' third_party/cyclonedds-python/clayer/pysertype.c

## 2. 但装上了也**用不了**：PEP 649 把它的注解机制打死了

`import cyclonedds` 直接崩：

    File "cyclonedds/idl/annotations.py", line 37, in __field_annotate
        raise TypeError(f"Member {pfield} is not defined.")
    TypeError: Member TRY_CONSTRUCT1 is not defined.

根因不在 C 层，而在 cyclonedds-python 的纯 Python 部分：它的 IDL 类靠
"类体执行时把字段塞进 `__annotations__`，再在类体里用 `annotate.position('X', 0)` 回头查"
这套反射机制。Python 3.14 默认启用 PEP 649（注解延迟求值），类体的
`__annotations__` 不再是执行期可读写的普通 dict，于是这套机制失效。
0.10.5 和 11.0.1 两个版本都一样。

**结论**：只要是 Python 3.14，`unitree_sdk2py`（它依赖 cyclonedds 的 Python 绑定）就用不了。
这不是配置问题，是上游还没适配。

## 3. 所以走 ROS 2

ROS 2 Lyrical Luth 的 Tier-1 平台就是 Ubuntu 26.04，官方 apt 包里的
`rclpy` 是给 Python 3.14 编的，`rmw_cyclonedds_cpp` 是 C++ 实现（不碰那套 Python 反射）。
所以：

    [MuJoCo SimNode] ←→ rclpy + rmw_cyclonedds_cpp ←→ rt/lowcmd / rt/lowstate ←→ 真机 Go2

DDS 那层、话题名、消息字段、CRC 全都一样 —— 唯一不同的是"谁来当 DDS 客户端"：
不是 unitree_sdk2py，而是 ROS 2 的 CycloneDDS RMW。

`transport/dds.py`（unitree_sdk2py）保留在仓库里，等哪天 Python 降到 3.12/3.13 的
环境可以直接用；本机验证走 `transport/ros2.py`。
