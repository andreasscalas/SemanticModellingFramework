#include "nonrigidfitting.h"
#include <vector>
#include <map>
#include <queue>
#include <algorithm>

NonRigidFitting::NonRigidFitting()
{

}

NonRigidFitting::NonRigidFitting(ExtendedTrimesh *template_mesh, ExtendedTrimesh *mesh_to_fit)
{
    this->template_mesh = template_mesh;
    this->mesh_to_fit = mesh_to_fit;
}


bool comp(std::pair<Annotation*, Annotation*> a, std::pair<Annotation*, Annotation*> b)
{
    if(a.first->getHierarchyLevel() <= b.first->getHierarchyLevel())
        return true;
    return false;
}

void NonRigidFitting::startFitting()
{
    std::vector<Annotation*> template_annotations = template_mesh->getAnnotations();
    std::vector<Annotation*> to_fit_annotations = mesh_to_fit->getAnnotations();
    std::vector<std::pair<Annotation*, Annotation*> > coupled_annotations;
    std::queue<Annotation*> Q;

    for(unsigned int i  = 0; i < to_fit_annotations.size(); i++)
        for(unsigned int j = 0; j < template_annotations.size(); j++)
            if(template_annotations[j]->getTag().compare(to_fit_annotations[i]->getTag()) == 0 &&
               template_annotations[j]->getType() == to_fit_annotations[i]->getType() &&
               template_annotations[j]->getType() == AnnotationType::Surface)
            {
                coupled_annotations.push_back(std::make_pair(template_annotations[j], to_fit_annotations[i]));
                break;
            }

    std::sort(coupled_annotations.begin(), coupled_annotations.end(), comp);
    unsigned int i = 0;
    for(std::vector<std::pair<Annotation*, Annotation*> >::iterator it = coupled_annotations.begin(); it != coupled_annotations.end(); it++)
    {
        std::cout<< "Couple n°: " << i++ << "First annotation: " << std::endl;
        it->first->print(std::cout);
        std::cout << std::endl << "Second annotation: " << std::endl;
        it->second->print(std::cout);
    }

}

ExtendedTrimesh *NonRigidFitting::getTemplateMesh() const
{
    return template_mesh;
}

void NonRigidFitting::setTemplateMesh(ExtendedTrimesh *value)
{
    template_mesh = value;
}

ExtendedTrimesh *NonRigidFitting::getMeshToFit() const
{
    return mesh_to_fit;
}

void NonRigidFitting::setMeshToFit(ExtendedTrimesh *value)
{
    mesh_to_fit = value;
}
