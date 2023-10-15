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

  // x: 光源区域的采样点 x
  // ws: 光源的方向 ws(wi)
  // nn: wi 跟光源区域(面)的法向
  // emit: 光亮
  Vector3f L_dir;
  Vector3f L_indir;
  Intersection p_light;
  float pdf_light;
  sampleLight(p_light, pdf_light);

  Vector3f p_p2light = p_light.coords - p.coords;

  auto t = intersect(Ray(p.coords, p_p2light.normalized()));

  // 判断light是否被遮挡
  if (t.distance >= p_p2light.norm() - EPSILON) {
    Vector3f ws = p_p2light.normalized();
    Vector3f NN = p_light.normal;
    Vector3f N = p.normal;
    Vector3f emit = p_light.emit;
    L_dir = emit * p.m->eval(ws, wo, N) * std::max(dotProduct(ws, N), 0.0f) *
            std::max(dotProduct(-ws, NN), 0.0f) /
            dotProduct(p_p2light, p_p2light) / pdf_light;
  }

  if (get_random_float() < RussianRoulette) {
    Vector3f wi = p.m->sample(wo, p.normal).normalized();
    float pdf_indir = p.m->pdf(wo, wi, p.normal);

    if (pdf_indir > EPSILON) {
      Intersection q = intersect(Ray(p.coords, wi));
      if (q.happened && q.m->hasEmission() == false) {
        L_indir = shade(q, -wi) * p.m->eval(wo, wi, p.normal) *
                  std::max(dotProduct(wi, p.normal), 0.0f) / pdf_indir /
                  RussianRoulette;
      }
    }
  }

  return L_dir + L_indir;
}
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const {
  // TO DO Implement Path Tracing Algorithm here

  // pdf(): 计算采样的概率密度值
  // eval(): 计算光线的 f_r 值
  Intersection inter = intersect(ray);
  if (inter.happened == false) {
    return {0, 0, 0};
  } else {
    return shade(inter, -ray.direction);
  }
}