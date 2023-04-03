/****************************************************************************
* DrawableTrimesh                                                           *
*                                                                           *
* Consiglio Nazionale delle Ricerche                                        *
* Istituto di Matematica Applicata e Tecnologie Informatiche                *
* Sezione di Genova                                                         *
* IMATI-GE / CNR                                                            *
*                                                                           *
* Authors: Andreas Scalas                                                   *
* Copyright(C) 2013: IMATI-GE / CNR                                         *
* All rights reserved.                                                      *
*                                                                           *
* This program is dual-licensed as follows:                                 *
*                                                                           *
* (1) You may use ImatiSTL as free software; you can redistribute it and/or *
* modify it under the terms of the GNU General Public License as published  *
* by the Free Software Foundation; either version 3 of the License, or      *
* (at your option) any later version.                                       *
* In this case the program is distributed in the hope that it will be       *
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty of    *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
* (2) You may use DrawableTrimesh as part of a commercial software. In this *
* case a proper agreement must be reached with the Authors and with         *
* IMATI-GE/CNR based on a proper licensing contract.                        *
****************************************************************************/

#ifndef DRAWABLEMESH_H
#define DRAWABLEMESH_H

#include <extendedtrimesh.h>
#include <annotation.h>

#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkPropAssembly.h>
#include <map>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>


class ExtendedTrimesh;
/**
 * @brief This class extends the TriMesh class defined in the ImatiSTL library
 * to allow the rendering of the mesh.
 */
class DrawableMesh : public ExtendedTrimesh {

    public:

        //Standard colors for various status of the mesh
        const unsigned char RED[3] =    {255,0,0};
        const unsigned char GREEN[3] =  {0,255,0};
        const unsigned char GRAY[3] =   {220,220,220};
        const unsigned char BLUE[3] =   {0,0,255};
        const unsigned char BLACK[3] =  {0,0,0};

        //Values for the info field f the mesh
        unsigned int INSIDE = 123;  //Used for marking the triangles which have already been identified for being in an annotaion

        DrawableMesh();
        DrawableMesh(DrawableMesh* m);
        DrawableMesh(ExtendedTrimesh* m);
        ~DrawableMesh();

        void init();
        /**
         * @brief draw Draws the mesh with/without its selection and annotations
         * @param assembly The assembly on which the mesh is drawn
         */
        void draw(vtkSmartPointer<vtkPropAssembly> assembly);

        /**
         * @brief load Loads the mesh identified by filename
         * @param filename The path and filename of the mesh
         * @return error code, see ImatiSTL/TriMesh to further information
         */
        int load(const char* filename);

        /**
         * @brief update Updates the status of the mesh
         */
        void update();

        /**
         * @brief buildVTKStructure Creates the VTK data correspondance to the IMATI STL mesh
         */
        void buildVTKStructure();

        //Getter and setter functions
        bool getDrawWireframe() const;
        void setDrawWireframe(bool value);
		bool getDrawPoints() const;
		void setDrawPoints(bool value);
        bool getDrawable() const;
        void setDrawable(bool value);
        bool getDrawSurface() const;
        void setDrawSurface(bool value);
        bool getMeshModified() const;
        void setMeshModified(bool value);
        bool getAnnotationsModified() const;
        void setAnnotationsModified(bool value);
        vtkSmartPointer<vtkPoints> getPoints();
        vtkSmartPointer<vtkCellArray> getTriangles();
        vtkSmartPointer<vtkPropAssembly> getCanvas() const;
        void setCanvas(const vtkSmartPointer<vtkPropAssembly> &value);

        //Getter and setter functions of indirect information
        void setPointPosition(vtkIdType pid, double* p);
        void setSelectedPoints(std::map<unsigned long, bool> selectedPoints);
        void setSelectedLines(std::map<unsigned long, bool> selectedLines);
        void setSelectedTriangles(std::map<unsigned long, bool> selectedTriangles);
        void setSelectedAnnotations(std::map<unsigned int, bool> selectedAnnotations);
        void setColor(int r, int g, int b);

        vtkSmartPointer<vtkActor> getMeshSurfaceActor() const;

        unsigned long getEdgeId(IMATI_STL::Edge*) const;
        IMATI_STL::Edge* getEdge(unsigned long) const;

        vtkSmartPointer<vtkActor> getMeshWireframeActor() const;

        vtkSmartPointer<vtkActor> getMeshPointsActor() const;

        bool getDrawAnnotations() const;
        void setDrawAnnotations(bool value);

        double getAnnotationsOpacity();
        void setAnnotationsOpacity(double value);

        vtkSmartPointer<vtkPoints> getMeshPoints() const;
        void setMeshPoints(const vtkSmartPointer<vtkPoints> &value);

        bool getOnlyPointsPositionsModified() const;
        void setOnlyPointsPositionsModified(bool value);

protected:

        //Flags                                            //True if the mesh is a cage
        bool drawable;                                              //True if the mesh has to be showed
        bool drawSurface;                                           //True if the mesh surface has to be showed
        bool drawWireframe;                                         //True if the mesh wireframe has to be showed
        bool drawPoints;											//True if the mesh points has to be showed
        bool drawAnnotations;                                       //True if the mesh annotations has to be showed
        bool meshModified;                                          //True if the mesh has been deformed
        bool annotationsModified;                                   //True if the annotations list has been edited
        bool onlyPointsPositionsModified;                           //True only if the mesh has been transformed

        //Data structures
        vtkSmartPointer<vtkPoints> meshPoints;                      //Points data structure in VTK for the mesh
        vtkSmartPointer<vtkCellArray> meshEdges;                    //Edges data structure in VTK for the mesh
        vtkSmartPointer<vtkCellArray> meshTriangles;                //Triangles data structure in VTK for the mesh

        //Graphical data
        vtkSmartPointer<vtkUnsignedCharArray> meshVColors;          //Array of colors of the vertices of the mesh
        vtkSmartPointer<vtkUnsignedCharArray> meshEColors;          //Array of colors of the lines of the mesh
        vtkSmartPointer<vtkUnsignedCharArray> meshTColors;          //Array of colors of the triangles of the mesh
        vtkSmartPointer<vtkPropAssembly> canvas;                        //Canvas onto which the different structures are drawed (mesh, annotations...)
        vtkSmartPointer<vtkActor> meshSurfaceActor;                 //Actor containing the mesh
        vtkSmartPointer<vtkActor> meshPointsActor;					//Actor containing the mesh points
		vtkSmartPointer<vtkActor> meshWireframeActor;               //Actor containing the mesh wireframe

        std::map<IMATI_STL::Edge*, unsigned long> edgesId;
        std::map<unsigned long, IMATI_STL::Edge*> idEdges;


};

#endif // DRAWABLEMESH_H
