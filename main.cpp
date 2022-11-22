#include <vector>
#include <iostream>
#include <cmath>
#include <limits>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"




Model* model = NULL;
const int width = 800;
const int height = 800;
int *zbuffer = NULL;

Vec3f       eye(1, 1, 3);       // 相机位置
Vec3f    center(0, 0, 0);       // 相机看向的位置. 
Vec3f        up(0, 1, 0);

//视口变换，
//    用于将 NDC 坐标 [-1,1]^3 映射到 宽w, 高h 的平面上, 并保留 z; 
//    过程就是:
//          先 scale, 后 translate; 
//          先 scale:
//              x 坐标 scale  w/2 倍.      // 因为是 [-1, 1], scale w/2 倍后, 正好宽度为 w; 
//              y 坐标 scale  h/2 倍.
//              z 坐标以某种方式保留.       // 如果不变的话, 那 z 还是 [-1, 1], 但我们可以把它转化成 [0, 255]
//          后 translate:
//              由于 [-1, 1]^3 立方体在 scale 时, 原点不变, 所以我们还要作一下位移, 位移量就是 [w/2, h/2]
//    而由于这个过程是先 scale 然后 tranlation; 所以可以直接往 4x4 matrix 里填; 
//      PS: 如果是先 tranlation 后 rotate/scale 的话, 是不能直接填的, 只能用矩阵乘法。 
//          先 tranlation 后 rotate / scale 的一个例子是 : 计算 world space 坐标--->camera space 坐标 的 matrix transformation 时。
//   该矩阵应该为:
//   [
//      w/2    0    0    w/2
//       0    h/2   0    h/2
//       0     0    1     0
//       0     0    0     1
//   ]
// 
//   注意:我们在调用时, 传入的 w, h 不是 image 的 width, height, 而是 3/4 width, 3/4 height; 
//        即我们的视口变换矩阵把 NDC cube 缩放成 image 3/4 的大小, 于是我们想让图像依然在图片中心的话,
//        还需要加一点位移: [x, y] = [1/8 width,  1/8 height] 
//   另外:
//       由于 z 是从 [-1, 1] 转换成 [0, 255], 所以矩阵稍微有点变化, 变成 m[2][2] = 255.f / 2.f; m[2][3] = 255.f / 2.f;
//   最终是这样一个矩阵:
//   [
//      w/2    0    0    w/2+x
//       0    h/2   0    h/2+y
//       0     0  255/2  255/2
//       0     0    0     1
//   ]
Matrix viewport(int x, int y, int w, int h) {
    Matrix m = Matrix::identity(4);
    m[0][3] = x+w/2.f;
    m[1][3] = y+h/2.f;

    m[0][0] = w/2.f;
    m[1][1] = h/2.f;

    m[2][2] = 255.f / 2.f;
    m[2][3] = 255.f / 2.f;
    return m;
}

// 这里没有用 tinnyrenderer 推导出的 perspective matrix; 而是用的 opengl 的; 
// tinnyrenderer 那个也能用, 但要注意如果要用那个的话, model view matrix 也要变。 
// 现在这个 model view matrix 是变成 camera space, 把 camera 放在了原点, 
// 而 tinnyrenderer 那个不是, 那个是把 "相机看向的位置" 那个点放在原点的, 并把 camera 放在 z+ 轴方向上。 
// 只有这样才能用那个 perspective matrix; 所以我改成了更常见的 透视矩阵。 
// 推导过程:     //TODO 
Matrix projection()
{
    float l = -0.3;
    float r = -l;
    float t = 0.3;
    float b = -t;
    float n = 0.9f;
    float f = 10.f;

    Matrix proj = Matrix(4, 4);
    proj[0][0] =  (2 * n) / (r - l);
    proj[0][2] =  (r + l) / (r - l);
    proj[1][1] =  (2 * n) / (t-b);
    proj[1][2] =  (t + b) / (t - b);
    proj[2][2] = -(f + n) / (f - n);
    proj[2][3] = -(2*n*f) / (f - n);
    proj[3][2] = -1;
    proj[3][3] =  0;
    
    return proj;
}

// ModelView matrix: 
Matrix modelView(Vec3f eye, Vec3f target, Vec3f up) {
    //计算出z，根据z和up算出x，再算出y
    Vec3f z = (eye - target).normalize();
    Vec3f x = (up ^ z).normalize();
    Vec3f y = (z ^ x).normalize();
    Matrix rotation = Matrix::identity(4);
    Matrix translation = Matrix::identity(4);
    //***矩阵的第四列是用于平移的。因为观察位置从原点变为了center，所以需要将物体平移-center***
    for (int i = 0; i < 3; i++) {
        translation[i][3] = -eye[i];
    }
    
    // 令 rotation 4x4矩阵中的 3x3 的部分称为 R, 则有:    
    //                      不能直接写出来, 从歪着的 rotation 旋转到与 axis aligned 并不容易, 但反过来旋转却很简单。
    //                      反过来的旋转 R-1 是: 每列分别为 x, y, z 的 column vector; 所以我们最终要的
    //                      rotation R 是 (R-1)^-1 ,  所以是 (R-1) 的转置
    //                  [
    //                      x
    //                      y
    //                      z
    //                  ]
    for (int i = 0; i < 3; i++) { // 每行分别为 x row vector, y row vector, z row vector; 
        rotation[0][i] = x[i];
        rotation[1][i] = y[i];
        rotation[2][i] = z[i];
    }
    //这样乘法的效果是先平移物体，再旋转
    Matrix res = rotation * translation;
    return res;
}


// 重心坐标计算.
// 
//对于某个点 P 相对于三角形 ABC 的重心坐标(u, v, w) 有:
//  u + v + w = 1
//
//  P = (1 - v - w)A + vB + wC
//  P = A + vAB + wAC
//  0 = vAB + wAC + PA         // 左边是 0 向量
//  由上式得到这样一个 linear system :
//  vAB<x> +wAC<x> +PA<x> = 0		// AB<x> 为向量 AB 的 x 分量
//  vAB<y> +wAC<y> +PA<y> = 0
//  观察得知(v, w, 1) 与这两个向量同时垂直 : (ABx, ACx, PAx), (ABy, ACy, PAy)
//  于是, (v, w, 1) 就能从(ABx, ACx, PAx), (ABy, ACy, PAy)  的 cross product 中求出来.
//  而最终的 barycentric coordinates 就是 :
//  (u, v, w) = ((1 - v - w), v, w)
Vec3f barycentric(Vec3i A, Vec3i B, Vec3i C, Vec3i P) {
    Vec3i s[2];     //存这两个向量:  (ABx, ACx, PAx),   (ABy, ACy, PAy) 
    for (int i = 2; i--; ) {
        s[i][0] = B[i] - A[i];
        s[i][1] = C[i] - A[i];
        s[i][2] = A[i] - P[i];
    }
    Vec3f t = s[0] ^ s[1];      // t 就是     k*(v, w, 1),  所以 v = t.x/t.z;  w = u.y / u.z; 
    if (std::abs(t[2]) < 1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(-1, 1, 1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
    return Vec3f(1.f - (t.x + t.y) / t.z, t.x / t.z, t.y / t.z);
}

// 画三角形.  画某个 pixel 时, uv 坐标要通过 重心坐标 与三顶点的 uv 坐标进行插值。 
void triangle(Vec3i* pts, Vec2i* uv, TGAImage& image, int* zbuffer) {
    Vec2i bboxmin(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());        //包围盒的左下角点
    Vec2i bboxmax(-std::numeric_limits<int>::max(), -std::numeric_limits<int>::max());      //包围盒的右下角点
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            bboxmin[j] = std::min(bboxmin[j], pts[i][j]);
            bboxmax[j] = std::max(bboxmax[j], pts[i][j]);
        }
    }
    Vec3i P;
    TGAColor color;
    // 扫描包围盒覆盖的每一个 pixel, 如果在三角形内, 则上色。
    for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
        for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
            Vec3f c = barycentric(pts[0], pts[1], pts[2], P);       // 算出 P 的重心坐标
            P.z = std::max(0, std::min(255, int(pts[0].z * c.x + pts[1].z * c.y + pts[2].z * c.z + .5))); // clamping to 0-255 since it is stored in unsigned char
            if (c.x < 0 || c.y < 0 || c.z<0 ) continue;

            int idx = P.x + P.y * width;
            if (P.z > zbuffer[idx]) continue;

            zbuffer[idx] = P.z;

            Vec2i uvP = uv[0] * c.x + uv[1] * c.y + uv[2] * c.z;    // 该 pixel 的插值后的 uv
            TGAColor color = model->diffuse(uvP);
            image.set(P.x, P.y, color);
            
        }
    }
}


int main(int argc, char** argv) {

    model = new Model("models/african_head.obj");

    //zbuffer 全部初始化为 int max
    zbuffer = new int[width*height];
    for (int i=0; i<width*height; i++) {
        zbuffer[i] = std::numeric_limits<int>::max();
    }

    //ModelView 变换矩阵;  ---> camera space 坐标
    Matrix ModelView = modelView(eye, center, Vec3f(0, 1, 0));

    //投影矩阵-透视   ---> NDC 坐标, [-1, 1]^3 立方体.
    Matrix Projection = projection();

    //视口变换  ---> 屏幕坐标
    Matrix ViewPort   = viewport(width/8, height/8, width*3/4, height*3/4);


    //准备 image
    TGAImage image(width, height, TGAImage::RGB);
    //光栅化: 对于某一 camera, 对 model 的每一个面(三角形), 
    //              1) 通过 MVP transformation 算出其三个顶点的屏幕坐标, 
    //              2) 从 model 中读取三个顶点的 uv 坐标, 
    //              3) 然后传入 zbuffer(深度缓冲) 并在设备(image)上绘制三角形, 这里用的是 包围盒 方法, 也可以用扫描线。 
    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);

        Vec3i screen_coords[3];
        Matrix MVP = Projection * ModelView;
        for (int j=0; j<3; j++) {
            Vec3f v = model->vert(face[j]);     // 3d 坐标;     model space;
            Matrix m_v = Matrix(v);             // 4d 齐次坐标;  model space; 
            screen_coords[j] =  Vec3f(ViewPort * MVP * m_v);    // 2d 坐标 + 深度信息:  screen space;  
        }
        Vec2i uv[3];
        for (int k = 0; k < 3; k++) {
            uv[k] = model->uv(i, k);
        }
        triangle(screen_coords, uv, image, zbuffer);
    }
    image.flip_vertically();
    image.write_tga_file("output.tga");

    delete model;
    delete [] zbuffer;
    return 0;
}

