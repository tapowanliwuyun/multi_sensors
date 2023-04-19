1. 使用说明
1.1 文件存放位置
1.1.1 如果是ros工程，就放在工程包
将 tic_toc.h 文件放在当前工程下 的 include 文件夹下 

1.1.2 如果是c++工程 ，就放在和cpp文件一样的目录下


1.2 引用说明
#include "tic_toc.h"

1.3 使用说明 

```cpp

    TicToc t_whole;

    printf("scan registration time %f ms *************\n", t_whole.toc());
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");

```
