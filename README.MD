参考 [tinnyrenderer ](https://github.com/ssloy/tinyrenderer.git),  主要逻辑在 main.cpp, 矩阵计算的代码用的现成的; 没有使用 tinnyrenderer 推导出的透视投影矩阵, 换了另外一个透视投影矩阵。

在 visual studio 中打开后, 直接运行即可, 会生成一个图片文件 output.tga;  模型文件放在 models 文件夹下, 是 WaveFront 格式.

另一套 transformation matrix 在分支 "another_set"
