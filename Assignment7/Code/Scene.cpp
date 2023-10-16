//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH() {
  printf(" - Generating BVH...\n\n");
  this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const {
  return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const {
  float emit_area_sum = 0;
  for (uint32_t k = 0; k < objects.size(); ++k) {
    if (objects[k]->hasEmit()) {
      emit_area_sum += objects[k]->getArea();
    }
  }
  float p = get_random_float() * emit_area_sum;
  emit_area_sum = 0;
  for (uint32_t k = 0; k < objects.size(); ++k) {
    if (objects[k]->hasEmit()) {
      emit_area_sum += objects[k]->getArea();
      if (p <= emit_area_sum) {
        objects[k]->Sample(pos, pdf);
        break;
      }
    }
  }
}

bool Scene::trace(const Ray &ray, const std::vector<Object *> &objects,
                  float &tNear, uint32_t &index, Object **hitObject) {
  *hitObject = nullptr;
  for (uint32_t k = 0; k < objects.size(); ++k) {
    float tNearK = kInfinity;
    uint32_t indexK;
    Vector2f uvK;
    if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
      *hitObject = objects[k];
      tNear = tNearK;
      index = indexK;
    }
  }

  return (*hitObject != nullptr);
}

Vector3f Scene::shade(const Intersection &p, const Vector3f &wo) const {
  if (p.m->hasEmission()) {
    return p.m->getEmission();
  }

  Vector3f L_dir;    // 直接光照
  Vector3f L_indir;  // 间接光照
  Intersection x;    // 光源上的采样点
  float pdf_light;   // 采样x点的概率密度
  sampleLight(x, pdf_light);
  Vector3f ws = (x.coords - p.coords).normalized();  // 指向光源采样点x的wi
  Vector3f NN = x.normal;  // 光线采样点x的法向
  Vector3f N = p.normal;   // 点p的法向
  Vector3f emit = x.emit;  // 光源采样点x的亮度
  // 判断light是否被遮挡
  if ((intersect(Ray(p.coords, ws)).coords - x.coords).norm() <= 0.01) {
    L_dir = emit * p.m->eval(ws, wo, N) * std::max(dotProduct(ws, N), 0.0f) *
            std::max(dotProduct(-ws, NN), 0.0f) /
            std::pow((x.coords - p.coords).norm(), 2) / pdf_light;
  }

  if (get_random_float() < RussianRoulette) {
    Vector3f wi = p.m->sample(wo, N).normalized();  // 随机采样wi
    Intersection q = intersect(Ray(p.coords, wi));  // p向wi方向发射光线的交点q
    float pdf_indir = p.m->pdf(wo, wi, N);  // 采样方向wi的概率密度

    if (pdf_indir > EPSILON) {
      if (q.happened && q.m->hasEmission() == false) {
        L_indir = shade(q, -wi) * p.m->eval(wo, wi, N) *
                  std::max(dotProduct(wi, N), 0.0f) / pdf_indir /
                  RussianRoulette;
      }
    }
  }

  return L_dir + L_indir;
}
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const {
  // TO DO Implement Path Tracing Algorithm here
  Intersection inter = intersect(ray);
  if (inter.happened == false) {
    return {0, 0, 0};
  } else {
    return shade(inter, -ray.direction);
  }
}