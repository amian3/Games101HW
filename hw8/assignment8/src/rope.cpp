#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D stride = (end - start) / (float)(num_nodes - 1); 
        for(int i = 0;i < num_nodes; ++i)
        {
            
            Mass* tempm = new Mass(start + i * stride, node_mass, false);
            masses.push_back(tempm);
        }
        for(int i = 0;i < num_nodes - 1; ++i)
        {
            Spring* temps = new Spring(masses[i], masses[i + 1], k);
            springs.push_back(temps);          
        }
//        Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            Vector2D ba = s->m2->position - s->m1->position;
            Vector2D F =  s->k * ba.unit() * (ba.norm() - s->rest_length);
            s->m1->forces += F;
            s->m2->forces -= F;
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
        }
        float damping_factor = 0.005f;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity * m->mass;
                m->forces += - m->velocity * damping_factor;
                Vector2D a = m->forces / m->mass;
                m->velocity = m->velocity + a * delta_t;
                m->position = m->position + m->velocity * delta_t;
                // TODO (Part 2): Add global damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            Vector2D ba = s->m2->position - s->m1->position;
            Vector2D F =  s->k * ba.unit() * (ba.norm() - s->rest_length);
            s->m1->forces += F;
            s->m2->forces -= F;
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
        }
        float damping_factor = 0.00005f;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity * m->mass;
                Vector2D a = m->forces / m->mass;
                m->velocity = m->velocity + a * delta_t;
                m->position = m->position + (1 - damping_factor) * (m->position - m->last_position) + a * delta_t * delta_t;
                m->last_position = temp_position;
                // TODO (Part 4): Add global Verlet damping
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
