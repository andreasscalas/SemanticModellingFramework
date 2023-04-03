#include "trianglehelper.h"

#ifndef ANSI_DECLARATORS
#define ANSI_DECLARATORS
#endif
#ifndef TRILIBRARY
#define TRILIBRARY
#endif
#ifndef REAL
#define REAL double
#endif
#ifndef VOID
#define VOID void
#endif
#include TRIANGLE_HEADER

#include <map>
using namespace std;
using namespace TriHelper;

TriangleHelper::TriangleHelper(std::vector<IMATI_STL::Point*> boundary, std::vector<std::vector<IMATI_STL::Point*> > holes){

    sliceMesh = new ExtendedTrimesh();
    this->boundary.insert(this->boundary.end(), boundary.begin(), boundary.end());
    this->holes.insert(this->holes.end(), holes.begin(), holes.end());


    launchTriangle();
}


ExtendedTrimesh *TriangleHelper::getSliceMesh() const{
    return sliceMesh;
}

void TriangleHelper::launchTriangle(){

    triangulateio in, out;

    vector<pair<unsigned int, unsigned int> > segments;

    int numberOfPoints = static_cast<int>(boundary.size() - 2);

    for(unsigned int i = 0; i < holes.size(); i++)
        numberOfPoints += holes[i].size() - 1;

    in.numberofpoints = numberOfPoints;
    in.pointlist      = static_cast<double*>(calloc(static_cast<size_t>(2 * in.numberofpoints), sizeof(double)));
    in.numberofpointattributes = 0;
    in.pointmarkerlist  = static_cast<int*>(calloc(static_cast<size_t>(in.numberofpoints), sizeof(int)));
    for(int vid = 0; vid < in.numberofpoints; vid++)
       in.pointmarkerlist[vid] = 1;

    unsigned int reachedID;
    for(unsigned int vid = 0; vid < static_cast<unsigned int>(boundary.size() - 2); vid++){

        in.pointlist[vid * 2  ]     = boundary[vid]->x;
        in.pointlist[vid * 2 + 1]   = boundary[vid]->z;
        if(vid > 0)
            segments.push_back(make_pair(vid - 1, vid));
        reachedID = vid;
    }

    segments.push_back(make_pair(reachedID, 0));
    reachedID++;
    for(vector<vector<IMATI_STL::Point*> >::iterator bit = holes.begin(); bit != holes.end(); bit++){
        unsigned int vid = reachedID;
        vector<IMATI_STL::Point*> outline = (*bit);
        for(vector<IMATI_STL::Point*>::iterator vit = outline.begin(); vit != outline.end() - 1; vit++){
            in.pointlist[vid * 2  ]     = (*vit)->x;
            in.pointlist[vid * 2 + 1]   = (*vit)->z;
            if(vid > reachedID)
                segments.push_back(make_pair(vid - 1, vid));

            vid++;
        }
        segments.push_back(make_pair(vid - 1, reachedID));
        reachedID = vid;
    }

    in.numberofsegments = static_cast<int>(segments.size());
    in.segmentlist      = static_cast<int*>(calloc(static_cast<size_t>(2 * in.numberofsegments), sizeof(int)));

    for(unsigned int i = 0; i < static_cast<unsigned int>(2 * in.numberofsegments); i += 2){
        in.segmentlist[i] = static_cast<int>(segments[i / 2].first);
        in.segmentlist[i + 1] = static_cast<int>(segments[i / 2].second);
    }

    in.segmentmarkerlist = nullptr;

    in.numberofholes = static_cast<int>(holes.size());
    in.holelist      = static_cast<double*>(calloc(static_cast<size_t>(2 * in.numberofholes), sizeof(double)));

    for(unsigned int hid = 0; hid < static_cast<unsigned int>(in.numberofholes); ++hid){

        vector<IMATI_STL::Point*> outline = holes[hid];
        IMATI_STL::Point v = (*(outline[1]) - *(outline[0])) * 1E-3;
        double vec[2] = {-v.z, v.x};
        IMATI_STL::Point middle = (*(outline[1]) + *(outline[0])) / 2;
        double innerPoint[2] = {vec[0] + middle.x, vec[1] + middle.z};
        double turningSign =    (outline[1]->z - outline[0]->z) * (innerPoint[0] - outline[0]->x) -
                                (outline[1]->x - outline[0]->x) * (innerPoint[1] - outline[0]->z);
        if(turningSign < 0){
            innerPoint[0] = -vec[0] + middle.x;
            innerPoint[1] = -vec[1] + middle.z;
        }

        in.holelist[hid * 2  ]      = innerPoint[0];
        in.holelist[hid * 2 + 1]    = innerPoint[1];
    }

    in.numberoftriangles          = 0;
    in.numberofcorners            = 0;
    in.numberoftriangleattributes = 0;
    in.trianglelist               = nullptr;
    in.triangleattributelist      = nullptr;
    in.numberofregions = 0;

    out.pointlist      = nullptr;
    out.trianglelist   = nullptr;
    out.segmentlist    = nullptr;

    std::string s = "pzBQ";

    triangulate(const_cast<char*>(s.c_str()), &in, &out, nullptr);

    for(int vid = 0; vid < out.numberofpoints; vid++){
        IMATI_STL::Vertex* v = sliceMesh->newVertex(out.pointlist[vid * 2], boundary[0]->y, out.pointlist[vid * 2 + 1]);
        sliceMesh->V.appendTail(v);
        idVertex[vid] = v;
    }

    for(int tid = 0; tid < out.numberoftriangles; tid++){
        IMATI_STL::ExtVertex* v1 = new IMATI_STL::ExtVertex(static_cast<IMATI_STL::Vertex*>(idVertex[out.trianglelist[tid * 3]]));
        IMATI_STL::ExtVertex* v2 = new IMATI_STL::ExtVertex(static_cast<IMATI_STL::Vertex*>(idVertex[out.trianglelist[tid * 3 + 1]]));
        IMATI_STL::ExtVertex* v3 = new IMATI_STL::ExtVertex(static_cast<IMATI_STL::Vertex*>(idVertex[out.trianglelist[tid * 3 + 2]]));
        sliceMesh->CreateTriangleFromVertices(v1, v2, v3);
    }
    sliceMesh->mergeCoincidentEdges();
    sliceMesh->checkGeometry();
    sliceMesh->checkConnectivity();
    sliceMesh->savePLY("prova.ply");
    free(in.pointlist);
    free(in.pointmarkerlist);
    free(in.segmentlist);
    free(in.holelist);
    free(out.pointlist);
    free(out.trianglelist);
    free(out.segmentlist);

}

#undef ANSI_DECLARATORS
#undef REAL
#undef VOID
#undef TRILIBRARY
