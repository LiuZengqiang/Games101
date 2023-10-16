#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

// 初始化 rope 对象
/**
 * @brief Construct a new Rope:: Rope object
 *
 * @param start 起始点
 * @param end
 * @param num_nodes
 * @param node_mass
 * @param k
 * @param pinned_nodes
 */
Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass,
           float k, vector<int> pinned_nodes) {
  // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and
  // containing `num_nodes` nodes.

  if (num_nodes == 0 || num_nodes == 1) return;
  // 绳子长度均匀分布
  // num_node 个节点, num_node-1根绳子
  for (int i = 0; i < num_nodes; i++) {
    Vector2D position = start + 1.0f * i * (end - start) / (num_nodes - 1);
    float mass = node_mass;

    if (i == 0) {
      Mass *m = new Mass(position, mass, false);
      masses.push_back(m);
    } else {
      Mass *a = masses.back();
      Mass *b = new Mass(position, mass, false);
      masses.push_back(b);
      Spring *s = new Spring(a, b, k);
      springs.push_back(s);
    }
  }

  //        Comment-in this part when you implement the constructor
  for (auto &i : pinned_nodes) {
    masses[i]->pinned = true;
  }
}

void Rope::simulateEuler(float delta_t, Vector2D gravity) {
  for (auto &s : springs) {
    // TODO (Part 2): Use Hooke's law to calculate the force on a node
    // 计算力: s->m1->forces, s->m2->forces;
    auto f = s->k * (s->m2->position - s->m1->position) /
             (s->m2->position - s->m1->position).norm() *
             ((s->m2->position - s->m1->position).norm() - s->rest_length);
    s->m1->forces += f;
    s->m2->forces -= f;
  }

  for (auto &m : masses) {
    if (!m->pinned) {
      // TODO (Part 2): Add the force due to gravity, then compute the new
      // velocity and position

      // 增加重力
      m->forces += gravity * m->mass;

      // TODO (Part 2): Add global damping
      if (false) {
        // 显式欧拉, 效果不好, 不稳定, 需要将-s参数设置为很大,例如8192,才能稳定
        // 增加摩擦力
        float k_d_global = 0.1;
        m->forces += -k_d_global * m->velocity;
        // 计算加速度：
        Vector2D a = m->forces / m->mass;
        // 计算位移：
        m->position += m->velocity * delta_t;
        // 计算速度：
        m->velocity += a * delta_t;

      } else {
        // 隐式欧拉(效果好, 稳定)
        float k_d_global = 0.01;
        m->forces += -k_d_global * m->velocity;
        // 计算加速度：
        Vector2D a = m->forces / m->mass;
        // 计算速度：
        m->velocity += a * delta_t;
        // 计算位移：
        m->position += m->velocity * delta_t;
      }
    }

    // Reset all forces on each mass
    m->forces = Vector2D(0, 0);
  }
}

void Rope::simulateVerlet(float delta_t, Vector2D gravity) {
  for (auto &s : springs) {
    // TODO (Part 3): Simulate one timestep of the rope using explicit
    // Verlet
    // （solving constraints)

    auto f = s->k * (s->m2->position - s->m1->position) /
             (s->m2->position - s->m1->position).norm() *
             ((s->m2->position - s->m1->position).norm() - s->rest_length);
    s->m1->forces += f;
    s->m2->forces -= f;
  }

  for (auto &m : masses) {
    if (!m->pinned) {
      Vector2D temp_position = m->position;
      // TODO (Part 3.1): Set the new position of the rope mass
      // TODO (Part 4): Add global Verlet damping
      // 增加重力:
      m->forces += gravity * m->mass;
      // 计算加速度:
      Vector2D a = m->forces / m->mass;
      // 根据 Verlet 方法计算位置:
      m->position = m->position +
                    (1.0f - 0.00005) * (m->position - m->last_position) +
                    a * delta_t * delta_t;

      m->last_position = temp_position;
    }
    m->forces = Vector2D(0, 0);
  }
}
}  // namespace CGL
