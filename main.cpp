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

Vec3f       eye(1, 1, 3);       // ���λ��
Vec3f    center(0, 0, 0);       // ��������λ��. 
Vec3f        up(0, 1, 0);

//�ӿڱ任��
//    ���ڽ� NDC ���� [-1,1]^3 ӳ�䵽 ��w, ��h ��ƽ����, ������ z; 
//    ���̾���:
//          �� scale, �� translate; 
//          �� scale:
//              x ���� scale  w/2 ��.      // ��Ϊ�� [-1, 1], scale w/2 ����, ���ÿ��Ϊ w; 
//              y ���� scale  h/2 ��.
//              z ������ĳ�ַ�ʽ����.       // �������Ļ�, �� z ���� [-1, 1], �����ǿ��԰���ת���� [0, 255]
//          �� translate:
//              ���� [-1, 1]^3 �������� scale ʱ, ԭ�㲻��, �������ǻ�Ҫ��һ��λ��, λ�������� [w/2, h/2]
//    ����������������� scale Ȼ�� tranlation; ���Կ���ֱ���� 4x4 matrix ����; 
//      PS: ������� tranlation �� rotate/scale �Ļ�, �ǲ���ֱ�����, ֻ���þ���˷��� 
//          �� tranlation �� rotate / scale ��һ�������� : ���� world space ����--->camera space ���� �� matrix transformation ʱ��
//   �þ���Ӧ��Ϊ:
//   [
//      w/2    0    0    w/2
//       0    h/2   0    h/2
//       0     0    1     0
//       0     0    0     1
//   ]
// 
//   ע��:�����ڵ���ʱ, ����� w, h ���� image �� width, height, ���� 3/4 width, 3/4 height; 
//        �����ǵ��ӿڱ任����� NDC cube ���ų� image 3/4 �Ĵ�С, ������������ͼ����Ȼ��ͼƬ���ĵĻ�,
//        ����Ҫ��һ��λ��: [x, y] = [1/8 width,  1/8 height] 
//   ����:
//       ���� z �Ǵ� [-1, 1] ת���� [0, 255], ���Ծ�����΢�е�仯, ��� m[2][2] = 255.f / 2.f; m[2][3] = 255.f / 2.f;
//   ����������һ������:
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

// ����û���� tinnyrenderer �Ƶ����� perspective matrix; �����õ� opengl ��; 
// tinnyrenderer �Ǹ�Ҳ����, ��Ҫע�����Ҫ���Ǹ��Ļ�, model view matrix ҲҪ�䡣 
// ������� model view matrix �Ǳ�� camera space, �� camera ������ԭ��, 
// �� tinnyrenderer �Ǹ�����, �Ǹ��ǰ� "��������λ��" �Ǹ������ԭ���, ���� camera ���� z+ �᷽���ϡ� 
// ֻ�������������Ǹ� perspective matrix; �����Ҹĳ��˸������� ͸�Ӿ��� 
// �Ƶ�����:     //TODO 
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
    //�����z������z��up���x�������y
    Vec3f z = (eye - target).normalize();
    Vec3f x = (up ^ z).normalize();
    Vec3f y = (z ^ x).normalize();
    Matrix rotation = Matrix::identity(4);
    Matrix translation = Matrix::identity(4);
    //***����ĵ�����������ƽ�Ƶġ���Ϊ�۲�λ�ô�ԭ���Ϊ��center��������Ҫ������ƽ��-center***
    for (int i = 0; i < 3; i++) {
        translation[i][3] = -eye[i];
    }
    
    // �� rotation 4x4�����е� 3x3 �Ĳ��ֳ�Ϊ R, ����:    
    //                      ����ֱ��д����, �����ŵ� rotation ��ת���� axis aligned ��������, ����������תȴ�ܼ򵥡�
    //                      ����������ת R-1 ��: ÿ�зֱ�Ϊ x, y, z �� column vector; ������������Ҫ��
    //                      rotation R �� (R-1)^-1 ,  ������ (R-1) ��ת��
    //                  [
    //                      x
    //                      y
    //                      z
    //                  ]
    for (int i = 0; i < 3; i++) { // ÿ�зֱ�Ϊ x row vector, y row vector, z row vector; 
        rotation[0][i] = x[i];
        rotation[1][i] = y[i];
        rotation[2][i] = z[i];
    }
    //�����˷���Ч������ƽ�����壬����ת
    Matrix res = rotation * translation;
    return res;
}


// �����������.
// 
//����ĳ���� P ����������� ABC ����������(u, v, w) ��:
//  u + v + w = 1
//
//  P = (1 - v - w)A + vB + wC
//  P = A + vAB + wAC
//  0 = vAB + wAC + PA         // ����� 0 ����
//  ����ʽ�õ�����һ�� linear system :
//  vAB<x> +wAC<x> +PA<x> = 0		// AB<x> Ϊ���� AB �� x ����
//  vAB<y> +wAC<y> +PA<y> = 0
//  �۲��֪(v, w, 1) ������������ͬʱ��ֱ : (ABx, ACx, PAx), (ABy, ACy, PAy)
//  ����, (v, w, 1) ���ܴ�(ABx, ACx, PAx), (ABy, ACy, PAy)  �� cross product �������.
//  �����յ� barycentric coordinates ���� :
//  (u, v, w) = ((1 - v - w), v, w)
Vec3f barycentric(Vec3i A, Vec3i B, Vec3i C, Vec3i P) {
    Vec3i s[2];     //������������:  (ABx, ACx, PAx),   (ABy, ACy, PAy) 
    for (int i = 2; i--; ) {
        s[i][0] = B[i] - A[i];
        s[i][1] = C[i] - A[i];
        s[i][2] = A[i] - P[i];
    }
    Vec3f t = s[0] ^ s[1];      // t ����     k*(v, w, 1),  ���� v = t.x/t.z;  w = u.y / u.z; 
    if (std::abs(t[2]) < 1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(-1, 1, 1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
    return Vec3f(1.f - (t.x + t.y) / t.z, t.x / t.z, t.y / t.z);
}

// ��������.  ��ĳ�� pixel ʱ, uv ����Ҫͨ�� �������� ��������� uv ������в�ֵ�� 
void triangle(Vec3i* pts, Vec2i* uv, TGAImage& image, int* zbuffer) {
    Vec2i bboxmin(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());        //��Χ�е����½ǵ�
    Vec2i bboxmax(-std::numeric_limits<int>::max(), -std::numeric_limits<int>::max());      //��Χ�е����½ǵ�
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            bboxmin[j] = std::min(bboxmin[j], pts[i][j]);
            bboxmax[j] = std::max(bboxmax[j], pts[i][j]);
        }
    }
    Vec3i P;
    TGAColor color;
    // ɨ���Χ�и��ǵ�ÿһ�� pixel, �������������, ����ɫ��
    for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
        for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
            Vec3f c = barycentric(pts[0], pts[1], pts[2], P);       // ��� P ����������
            P.z = std::max(0, std::min(255, int(pts[0].z * c.x + pts[1].z * c.y + pts[2].z * c.z + .5))); // clamping to 0-255 since it is stored in unsigned char
            if (c.x < 0 || c.y < 0 || c.z<0 ) continue;

            int idx = P.x + P.y * width;
            if (P.z > zbuffer[idx]) continue;

            zbuffer[idx] = P.z;

            Vec2i uvP = uv[0] * c.x + uv[1] * c.y + uv[2] * c.z;    // �� pixel �Ĳ�ֵ��� uv
            TGAColor color = model->diffuse(uvP);
            image.set(P.x, P.y, color);
            
        }
    }
}


int main(int argc, char** argv) {

    model = new Model("models/african_head.obj");

    //zbuffer ȫ����ʼ��Ϊ int max
    zbuffer = new int[width*height];
    for (int i=0; i<width*height; i++) {
        zbuffer[i] = std::numeric_limits<int>::max();
    }

    //ModelView �任����;  ---> camera space ����
    Matrix ModelView = modelView(eye, center, Vec3f(0, 1, 0));

    //ͶӰ����-͸��   ---> NDC ����, [-1, 1]^3 ������.
    Matrix Projection = projection();

    //�ӿڱ任  ---> ��Ļ����
    Matrix ViewPort   = viewport(width/8, height/8, width*3/4, height*3/4);


    //׼�� image
    TGAImage image(width, height, TGAImage::RGB);
    //��դ��: ����ĳһ camera, �� model ��ÿһ����(������), 
    //              1) ͨ�� MVP transformation ����������������Ļ����, 
    //              2) �� model �ж�ȡ��������� uv ����, 
    //              3) Ȼ���� zbuffer(��Ȼ���) �����豸(image)�ϻ���������, �����õ��� ��Χ�� ����, Ҳ������ɨ���ߡ� 
    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);

        Vec3i screen_coords[3];
        Matrix MVP = Projection * ModelView;
        for (int j=0; j<3; j++) {
            Vec3f v = model->vert(face[j]);     // 3d ����;     model space;
            Matrix m_v = Matrix(v);             // 4d �������;  model space; 
            screen_coords[j] =  Vec3f(ViewPort * MVP * m_v);    // 2d ���� + �����Ϣ:  screen space;  
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

