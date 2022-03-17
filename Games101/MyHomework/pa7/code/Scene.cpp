//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include <future>
#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection intersection = intersect(ray);
    Vector3f L = Vector3f(0);
    if(intersection.happened){
        if(depth == 0 && intersection.m->hasEmission()){
            return intersection.m->getEmission();
        }

        Vector3f wo = normalize(-ray.direction);
        Vector3f p = intersection.coords;
        Vector3f n = normalize(intersection.normal);

        float pdf_light = 0.0f;
        Intersection inter;
        sampleLight(inter, pdf_light);

        Vector3f x =inter.coords;
        Vector3f ws = normalize(x - p);
        Vector3f nn = normalize(inter.normal);

        bool blocked = (intersect(Ray(p, ws)).coords - x).norm() > EPSILON;
        Vector3f L_dir = Vector3f(0.0f);

        if(!blocked){
            float dist = (x-p).norm();
            L_dir = inter.emit * intersection.m->eval(wo, ws, n) * dotProduct(ws, n) * dotProduct(-ws, nn) / (dist * dist * pdf_light);
        }

        Vector3f L_indir = Vector3f(0.0f);
        float P_RR = get_random_float();

        if(P_RR < Scene::RussianRoulette){
            Vector3f wi = intersection.m->sample(wo,n);
            Ray r(p, wi);
            Intersection q = Scene::intersect(r);

            if(q.happened){
                bool hitIsEmitting = !q.m->hasEmission();
                if(hitIsEmitting){
//                    auto future = std::async(std::launch::async, &Scene::castRay, this, r, depth + 1);
//                    L_indir = future.get() * intersection.m->eval(wo,wi,N)*dotProduct(wi,N)/(intersection.m->pdf(wo,wi,N) * Scene::RussianRoulette);
                    L_indir = castRay(r, depth + 1) * intersection.m->eval(wo,wi,n)*dotProduct(wi, n)/(intersection.m->pdf(wo,wi, n) * Scene::RussianRoulette);
                }
            }
        }
        L = L_dir + L_indir;
    }
    return L;
}



