//
// Created by Göksu Güvendiren on 2019-05-14.
//

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





/*


Vector3f Scene::castRay(const Ray &ray, int depth) const
{
     Vector3f L_dir = { 0,0,0 };
     Vector3f L_indir = { 0,0,0 };
 
     Intersection intersection = Scene::intersect(ray);
     if (!intersection.happened)
     {
         return {};
     }
     //打到光源
     if (intersection.m->hasEmission())
     return intersection.m->getEmission();
 
     //打到物体后对光源均匀采样
     Intersection lightpos;
     float lightpdf = 0.0f;
     sampleLight(lightpos, lightpdf);//获得对光源的采样，包括光源的位置和采样的pdf
 
     Vector3f collisionlight = lightpos.coords - intersection.coords;
     float dis = std::pow(collisionlight.norm(), 2);
     Vector3f collisionlightdir = collisionlight.normalized();
     Ray objray(intersection.coords, collisionlightdir);
 
     Intersection ishaveobj = Scene::intersect(objray);
     //L_dir = L_i * f_r * cos_theta * cos_theta_x / |x - p | ^ 2 / pdf_light
     if (ishaveobj.distance - collisionlight.norm() > -EPSILON)//说明之间没有遮挡
     L_dir = lightpos.emit * intersection.m->eval(ray.direction, collisionlightdir, intersection.normal) * dotProduct(collisionlightdir, intersection.normal) * dotProduct(-collisionlightdir, lightpos.normal) / dis / lightpdf;
 	 //打到物体后对半圆随机采样使用RR算法
     if (get_random_float() > RussianRoulette)
     return L_dir;
 
     Vector3f w0 = intersection.m->sample(ray.direction, intersection.normal).normalized();
     Ray objrayobj(intersection.coords, w0);
     Intersection islight = Scene::intersect(objrayobj);
     // shade(q, wi) * f_r * cos_theta / pdf_hemi / P_RR
     if (islight.happened && !islight.m->hasEmission())
     {
         float pdf = intersection.m->pdf(ray.direction, w0, intersection.normal);
         L_indir = castRay(objrayobj, depth + 1) * intersection.m->eval(ray.direction, w0, intersection.normal) * dotProduct(w0, intersection.normal) / pdf / RussianRoulette;
     }
 
     return L_dir + L_indir;
}
*/

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection intersection = Scene::intersect(ray);
    Material *m = intersection.m;
    Object *hitObject = intersection.obj;
    Vector3f hitColor = this->backgroundColor;
    if(!intersection.happened)
    {
        return {};
    }
    if(intersection.m->hasEmission())
    {
        return intersection.m->getEmission();
    }
    else{
        Vector3f L_dir, L_indir;
        float lightpdf;
        Intersection singlelight;
        sampleLight(singlelight, lightpdf);
        Vector3f dir = singlelight.coords - intersection.coords;
        Vector3f ori = intersection.coords;
        Ray lightray(ori, dir.normalized());
        Intersection lightcross = Scene::intersect(lightray);
        if(lightcross.distance - dir.norm() > - EPSILON)
        {
            Vector3f fr = intersection.m->eval(ray.direction, dir.normalized(), intersection.normal);
            L_dir = singlelight.emit * fr * dotProduct(-ray.direction, intersection.normal) * dotProduct(intersection.normal, dir.normalized()) / (std::pow(dir.norm(), 2) * lightpdf);
        }
        if(get_random_float() > RussianRoulette)
        {
            return L_dir;
        }
        float spherepdf;
        Vector3f spherecase = intersection.m->sample(ray.direction, intersection.normal).normalized();
        Ray sphereemit(ori, spherecase);
        Intersection spherecross = Scene::intersect(sphereemit);
        if(spherecross.happened && !spherecross.m->hasEmission())
        {
            Vector3f fr = intersection.m->eval(ray.direction, spherecase, intersection.normal);
            spherepdf = intersection.m->pdf(ray.direction, spherecase, intersection.normal);
            L_indir = castRay(sphereemit, depth + 1) * fr * dotProduct(spherecase, intersection.normal)/ (spherepdf * RussianRoulette);
        }
        

        return L_indir + L_dir;
    }
}

