---
title: GAMES101 作业7 路径追踪 踩坑指南
date: 2024-04-16 13:35:32
tags: 图形学
---


<meta name="referrer" content="no-referrer"/>


首先回顾路径追踪的原理，如下图

<img src="https://img2023.cnblogs.com/blog/1928276/202404/1928276-20240416121322581-1913642501.png" alt="1713229011326" style="zoom:50%;" />


## 基本思想

wo是射向眼镜（相机）的光线，包含来自光源的直接光照ws，来自其他物体的间接光照wi两部分。

在实现path tracing时，我们考虑的是黄色线的方向，即光线从相机射向p点（实际上是从p点射向相机），然后通过多次随机采样从p点射出（实际上是射向p点）的光线得到该像素点的真实颜色。

为了提高效率，将射向p的光线分为ws(光源)和wi（其他物体）计算。由于wi、ws分开计算，因此如果ws被物体挡住,或者wi打到光源均不计算。

wi需要递归计算，通过神奇的Russian Roulette在减少递归层数的同时保持光照的期望不变。

然后按照作业指南上的伪代码写就可以了

<img src="https://img2023.cnblogs.com/blog/1928276/202404/1928276-20240416121328730-1409855486.png" alt="1713230279835" style="zoom:80%;" />



## 注意事项

- 右墙壁发黑：检查Bound3::IntersectP, `return t_enter <= t_exit && t_exit >= 0;` 就可以
- 小正方体右上角有三角形黑块:检查Triangle::getIntersectionin Triangle.hpp，当时间小于0时不能判定为相交
- 多线程：注意framebuffer的下标应该由m改为直接用i和j计算。CMakeLists.txt加一行 `TARGET_LINK_LIBRARIES(RayTracing pthread)`就好。

## 代码

### 多线程优化

```cpp
    // change the spp value to change sample ammount
    int spp = 32; // default:16
    std::cout << "SPP: " << spp << "\n";

    // for (uint32_t j = 0; j < scene.height; ++j) {
    //     for (uint32_t i = 0; i < scene.width; ++i) {
    //         // generate primary ray direction
    //         float x = (2 * (i + 0.5) / (float)scene.width - 1) *
    //                   imageAspectRatio * scale;
    //         float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

    //         Vector3f dir = normalize(Vector3f(-x, y, 1));
    //         for (int k = 0; k < spp; k++){
    //             framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
    //         }
    //         m++;
    //     }
    //     UpdateProgress(j / (float)scene.height);
    // }
    // UpdateProgress(1.f);

    const int thread_cnt = 12;
    int finished_thread = 0;
    int finished_width = 0;
    std::mutex mtx;
  
    printf("%d %d\n", scene.height, scene.width);
    auto multiThreadCastRay = [&](uint32_t y_min, uint32_t y_max) 
    {
        printf("start %d %d\n", y_min, y_max);
        for (uint32_t j = y_min; j <= y_max; ++j) {
            for (uint32_t i = 0; i < scene.width; ++i) {
                // generate primary ray direction
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                        imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));
                for (int k = 0; k < spp; k++) {
                    framebuffer[scene.width * j + i] += scene.castRay(Ray(eye_pos, dir), 0) / spp; 
                }
            }
            //printf("%d\n", j);
            //UpdateProgress(j / (float)scene.height);
            mtx.lock();
            UpdateProgress(++finished_width * 1.0 / scene.width);
            mtx.unlock();
        }
        printf("ok %d %d\n", y_min, y_max);
    };
    int block = scene.height / thread_cnt + (scene.height % thread_cnt != 0);
    std::thread th[thread_cnt];
    for (int i = 0; i < thread_cnt; i++) {
        th[i] = std::thread(multiThreadCastRay, i * block, std::min((i + 1) * block - 1, scene.height));
    }
    for (int i = 0; i < thread_cnt; i++) th[i].join();
    UpdateProgress(1.0);

```

### 路径追踪

```cpp
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    /*
    shade(p, wo)
        sampleLight(inter , pdf_light)
        Get x, ws, NN, emit from inter
        Shoot a ray from p to x
        If the ray is not blocked in the middle
            L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws,
            NN) / |x-p|^2 / pdf_light
  
  
        L_indir = 0.0
        //Test Russian Roulette with probability RussianRoulette
        wi = sample(wo, N)
        Trace a ray r(p, wi)
        If ray r hit a non-emitting object at q
            L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N)
            / pdf(wo, wi, N) / RussianRoulette
  
        Return L_dir + L_indir
    */

    Vector3f L_dir(0, 0, 0), L_indir(0, 0, 0);
    //ray wo is screen to p, now find p and see if already hit light
    Ray wo = ray;
    Intersection p_inter = this->intersect(wo);
    //if hit nothing
    if (!p_inter.happened) return L_dir;
    //if hit light source
    if (p_inter.m->hasEmission()) return p_inter.m->getEmission();

    //otherwise, it hit a object

    //sampleLight(inter , pdf_light)
    //uniformly sample x from all LIGHTS and get its pdf
    Intersection x_inter; float x_pdf;
    sampleLight(x_inter, x_pdf);

    //Get x, ws, Nx, emit from inter 
    //ws is from p to x(light), Np is at p, Nx is at x(light)
    Vector3f p = p_inter.coords;
    Vector3f x = x_inter.coords;
    Vector3f Np = p_inter.normal;
    Vector3f Nx = x_inter.normal;
    Vector3f emit = x_inter.emit;  

    //Shoot a ray (ws) from p to x(light) 
    Vector3f ws_dir = (x - p).normalized();
    Ray ws(p, ws_dir);
    Intersection ws_inter = this->intersect(ws);

    // If the ray is NOT blocked in the middle
    //         L_dir = emit * eval(wo, ws, N) * dot(ws, N) * dot(ws,
    //         NN) / |x-p|^2 / pdf_light
    // Else L_dir = 0.0

    //calc length of p - x and ws_inter to see if it is blocked
    float px_dis = (x - p).norm(), ws_dis = ws_inter.distance;
    if (px_dis - ws_dis < 0.001) {
        L_dir = emit 
        * p_inter.m->eval(wo.direction, ws.direction, Np)
        * dotProduct(ws.direction, Np)      //all vectors were nomorlized
        * dotProduct(-ws.direction, Nx)     //so dot product is cosine
        / pow(px_dis, 2)
        / x_pdf;
    } // else L_dir = 0; no need
  
    // Now calculate L_indir
    // Test Russian Roulette with probability RussianRoulette
    float P_rand = get_random_float();
    if (P_rand < RussianRoulette) {
        //wi = sample(wo, N)
        //wi is from p to q
        Vector3f wi_dir = p_inter.m->sample(wo.direction, Np).normalized();
        Ray wi(p_inter.coords, wi_dir);
        // Trace a ray r(p, wi)
        // If ray r hit a non-emitting object at q
        //     L_indir = shade(q, wi) * eval(wo, wi, N) * dot(wi, N)
        //     / pdf(wo, wi, N) / RussianRoulette
        Intersection wi_inter = this->intersect(wi);
        if (wi_inter.happened && !(wi_inter.m->hasEmission())) {
            L_indir = castRay(wi, depth + 1)
            * p_inter.m->eval(wo.direction, wi.direction, Np)
            * dotProduct(wi.direction, Np)
            / p_inter.m->pdf(wo.direction, wi.direction, Np)
            / RussianRoulette;
        }
    }
    return L_dir + L_indir;
}
```
