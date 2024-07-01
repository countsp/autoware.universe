## colcon build时候报错

![Screenshot from 2024-07-01 13-40-36](https://github.com/countsp/autoware.universe/assets/102967883/3123646a-de91-4124-8844-cc128513afe0)

**解决方法**
将CmakeList.txt中的 C_STANDARD 设置为 11 或 99，这是 CMake 3.16 支持的最新标准。

```
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
```
