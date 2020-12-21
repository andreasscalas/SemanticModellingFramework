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

#include "extendedtrimesh.h"
#include "annotation.h"
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkTriangle.h>
#include <vtkLine.h>
#include <vtkPolyDataMapper.h>
#include <vtkCellArray.h>
#include <vtkProperty.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkAssembly.h>
#include <vtkProperty.h>
#include <vtkCellData.h>
#include <map>
#include <stdio.h>
#include <string>

#define VTK_CREATE(type, name) \
vtkSmartPointer<type> name = vtkSmartPointer<type>::New()

class ExtendedTrimesh;
class Annotation;
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

        //Values for the info field f the mesh
        const unsigned int INSIDE =   123;  //Used for marking the triangles which have already been identified for being in an annotaion

        DrawableMesh();
        /**
         * @brief draw Draws the mesh with/without its selection and annotations
         * @param assembly The assembly on which the mesh is drawn
         */
        void draw(vtkSmartPointer<vtkAssembly> assembly) const;

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
        bool getIsCage() const;
        void setIsCage(bool value);
        bool getDrawWireframe() const;
        void setDrawWireframe(bool value);
        bool getDrawable() const;
        void setDrawable(bool value);
        bool getMeshModified() const;
        void setMeshModified(bool value);
        bool getColorModified() const;
        void setColorModified(bool value);
        bool getAnnotationsModified() const;
        void setAnnotationsModified(bool value);
        std::string getFilename() const;
        vtkSmartPointer<vtkPoints> getPoints();
        vtkSmartPointer<vtkCellArray> getTriangles();

        vtkSmartPointer<vtkAssembly> getCanvas() const;
        void setCanvas(const vtkSmartPointer<vtkAssembly> &value);

        //Getter and setter functions of indirect information
        void setPointPosition(vtkIdType pid, double* p);
        void setSelectedPoints(std::map<long, bool> selectedPoints);
        void setSelectedTriangles(std::map<long, bool> selectedTriangles);
        void setColor(int r, int g, int b);

        vtkSmartPointer<vtkActor> getMeshActor() const;

protected:

        //Flags
        bool isCage;                                                //True if the mesh is a cage
        bool drawWireframe;                                         //True if the mesh has to be showed in wirefrime mode
        bool drawable;                                              //True if the mesh has to be showed in flat surface mode
        bool meshModified;                                          //True if the mesh has been deformed
        bool annotationsModified;                                   //True if the annotations list has been edited
        bool colorModified;                                         //True if the clor has been changed

        //Properties of the mesh
        std::string filename;                                       //Name of the file in which the mesh is stored

        //Data structures
        vtkSmartPointer<vtkPoints> meshPoints;                      //Points data structure in VTK for the mesh
        vtkSmartPointer<vtkCellArray> meshTriangles;                //Triangles data structure in VTK for the mesh
        vtkSmartPointer<vtkCellArray> annotatedTriangles;           //Data structure in VTK for the annotated triangles
        vtkSmartPointer<vtkPolyData> vtkMesh;                       //Data structure to store the mesh in the VTK format

        //Graphical data
        vtkSmartPointer<vtkUnsignedCharArray> meshVColors;          //Array of colors of the vertices of the mesh
        vtkSmartPointer<vtkUnsignedCharArray> meshTColors;          //Array of colors of the triangles of the mesh
        vtkSmartPointer<vtkUnsignedCharArray> annotationsTColors;   //Array of colors of the triangles of the annotations
        vtkSmartPointer<vtkAssembly> canvas;                        //Canvas onto which the different structures are drawed (mesh, annotations...)
        vtkSmartPointer<vtkActor> meshActor;                        //Actor containing the mesh

};

#endif // DRAWABLEMESH_H
