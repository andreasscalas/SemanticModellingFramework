#include "semanticgraphinteractionstyle.h"

#include <string>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkRenderedAreaPicker.h>
#include <vtkExtractGeometry.h>
#include <vtkImplicitFunction.h>
#include <vtkPlanes.h>
#include <vtkCamera.h>
#include <vtkVertexGlyphFilter.h>

SemanticGraphInteractionStyle::SemanticGraphInteractionStyle()
{
    this->model = new QStringListModel();
    this->PointPicker = vtkSmartPointer<vtkPointPicker>::New();
    this->CellPicker = vtkSmartPointer<vtkCellPicker>::New();

}

void SemanticGraphInteractionStyle::OnRightButtonDown()
{
    vtkInteractorStyleRubberBand2D::OnRightButtonDown();
}

void SemanticGraphInteractionStyle::OnRightButtonUp()
{
    vtkInteractorStyleRubberBand2D::OnRightButtonUp();
}

void SemanticGraphInteractionStyle::OnMouseMove()
{
    int x = this->Interactor->GetEventPosition()[0];
    int y = this->Interactor->GetEventPosition()[1];
    this->FindPokedRenderer(x, y);
    if (this->Interaction == PANNING || this->Interaction == ZOOMING)
    {
        vtkRenderWindowInteractor* rwi = this->GetInteractor();
        int lastPt[] = {0, 0};
        rwi->GetLastEventPosition(lastPt);
        int curPt[] = {0, 0};
        rwi->GetEventPosition(curPt);

        vtkCamera* camera = this->CurrentRenderer->GetActiveCamera();
        double lastScale = 2.0 * camera->GetParallelScale() / this->CurrentRenderer->GetSize()[1];
        double lastFocalPt[] = {0, 0, 0};
        camera->GetFocalPoint(lastFocalPt);
        double lastPos[] = {0, 0, 0};
        camera->GetPosition(lastPos);

        if (this->Interaction == PANNING)
        {
          double delta[] = {0, 0, 0};
          delta[0] = -lastScale*(curPt[0] - lastPt[0]);
          delta[1] = -lastScale*(curPt[1] - lastPt[1]);
          delta[2] = 0;
          camera->SetFocalPoint(lastFocalPt[0] + delta[0], lastFocalPt[1] + delta[1], lastFocalPt[2] + delta[2]);
          camera->SetPosition(lastPos[0] + delta[0], lastPos[1] + delta[1], lastPos[2] + delta[2]);
          this->InvokeEvent(vtkCommand::InteractionEvent);
          rwi->Render();
        }
        else if (this->Interaction == ZOOMING)
        {
          double motion = 10.0;
          double dyf = motion*(curPt[1] - lastPt[1])/this->CurrentRenderer->GetCenter()[1];
          double factor = pow(1.1, dyf);
          camera->SetParallelScale(camera->GetParallelScale() / factor);
          this->InvokeEvent(vtkCommand::InteractionEvent);
          rwi->Render();
        }
    } else if (this->Interaction == SELECTING)
    {
        this->EndPosition[0] = x;
        this->EndPosition[1] = y;
        int *size = this->Interactor->GetRenderWindow()->GetSize();
        if (this->EndPosition[0] > (size[0] - 1))
            this->EndPosition[0] = size[0] - 1;
        if (this->EndPosition[0] < 0)
            this->EndPosition[0] = 0;
        if (this->EndPosition[1] > (size[1] - 1))
            this->EndPosition[1] = size[1] - 1;
        if (this->EndPosition[1] < 0)
            this->EndPosition[1] = 0;
        vtkPlanes* frustum = static_cast<vtkRenderedAreaPicker*>(this->GetInteractor()->GetPicker())->GetFrustum();

        vtkSmartPointer<vtkExtractGeometry> extractGeometry = vtkSmartPointer<vtkExtractGeometry>::New();
        extractGeometry->SetImplicitFunction(static_cast<vtkImplicitFunction*>(frustum));

        extractGeometry->SetInputConnection(graphToPolydataFilter->GetOutputPort());
        extractGeometry->Update();

        vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
        glyphFilter->SetInputConnection(extractGeometry->GetOutputPort());
        glyphFilter->Update();

        vtkSmartPointer<vtkPolyData> selected = glyphFilter->GetOutput();

        if(selected->GetNumberOfPoints() > 0){
            for(unsigned int i = 0; i < selected->GetNumberOfPoints(); i++){
                double *point = selected->GetPoint(i);
                std::cout << "(" << point[0] << "," << point[1] << "," << point[2] << ")" << std::endl << std::flush;
            }

            for(unsigned int i = 0; i < selected->GetNumberOfCells(); i++){
                vtkCell* cell = selected->GetCell(i);
                vtkIdList* cellsid = cell->GetPointIds();
                for(unsigned int j = 0; j < cellsid->GetNumberOfIds(); j++)
                    std::cout << "Cell n. " << j << ": " << cellsid->GetId(j) << std::endl << std::flush;
            }
        }
        this->InvokeEvent(vtkCommand::InteractionEvent);
        this->RedrawRubberBand();
    } else {
        // Get the selected point

        this->PointPicker->SetTolerance(0.01);
        this->PointPicker->Pick(x, y, 0,
                 this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());

        this->listView->reset();
        delete model;
        model = new QStringListModel(listView);
        QStringList list;

        vtkIdType pickedPointId = this->PointPicker->GetPointId();
        if(pickedPointId >= 0 && pickedPointId < static_cast<long>(graph->getNodes().size()))
        {
            label->setText("Annotation info");

            GraphTemplate::Node<Annotation*>* node = graph->getNodes()[static_cast<unsigned int>(pickedPointId)];

            std::stringstream stream;
            std::string line;
            node->getData()->print(stream);
            while(!stream.eof()){
                std::string line;
                getline(stream, line, '\n');
                list << line.c_str();
            }

        } else {

            this->CellPicker->SetTolerance(0.005);
            this->CellPicker->Pick(x, y, 0,
                     this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());

            vtkIdType pickedCellId = this->CellPicker->GetCellId();
            if(pickedCellId >= 0)
            {
                label->setText("Relationship info");

                GraphTemplate::Arc<Annotation*>* arc = graph->getArcs()[static_cast<unsigned int>(pickedCellId)];

                std::stringstream stream;
                list << ("type: " + arc->getLabel()).c_str();
                list << ("weight: " + std::to_string(arc->getWeight())).c_str();
                list << ("involved annotations: " +
                        std::to_string(arc->getN1()->getData()->getId()) + " and " +
                        std::to_string(arc->getN2()->getData()->getId())).c_str();


            }
        }
        this->model->setStringList(list);
        this->listView->setModel(model);
        this->listView->update();
    }
}

void SemanticGraphInteractionStyle::OnLeftButtonDown()
{

    if(this->Interaction == NONE)
    {
        this->Interaction = SELECTING;
        // Get the selected point
        int x = this->Interactor->GetEventPosition()[0];
        int y = this->Interactor->GetEventPosition()[1];
        this->FindPokedRenderer(x, y);
        vtkRenderWindow *renWin = this->Interactor->GetRenderWindow();

        this->StartPosition[0] = x;
        this->StartPosition[1] = y;
        this->EndPosition[0] = x;
        this->EndPosition[1] = y;

        this->PixelArray->Initialize();
        this->PixelArray->SetNumberOfComponents(4);
        int *size = renWin->GetSize();
        this->PixelArray->SetNumberOfTuples(size[0] * size[1]);

        renWin->GetRGBACharPixelData(0, 0, size[0] - 1, size[1] - 1, 1, this->PixelArray);

        this->PointPicker->SetTolerance(0.005);
        this->PointPicker->Pick(x, y, 0,
                 this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());

        vtkIdType pickedPointId = this->PointPicker->GetPointId();
        if(pickedPointId >= 0 && pickedPointId < static_cast<long>(graph->getNodes().size()))
        {
            std::vector<unsigned int>::iterator it = std::find(selectedNodes.begin(), selectedNodes.end(), pickedPointId);
            if(it == selectedNodes.end()){
                selectedNodes.push_back(static_cast<unsigned int>(pickedPointId));
                nodesColor->InsertValue(pickedPointId, 1);
            }


            for(unsigned int i = 0; i < selectedEdges.size(); i++)
                edgesColor->InsertValue(selectedEdges[i], 0);

            selectedEdges.clear();

        } else {

            this->CellPicker->SetTolerance(0.005);
            this->CellPicker->Pick(x, y, 0,
                     this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());

            vtkIdType pickedCellId = this->CellPicker->GetCellId();
            if(pickedCellId >= 0)
            {
                std::vector<unsigned int>::iterator it = std::find(selectedEdges.begin(), selectedEdges.end(), pickedCellId);
                if(it == selectedEdges.end()){
                    selectedEdges.push_back(static_cast<unsigned int>(pickedCellId));
                    edgesColor->InsertValue(pickedCellId, 1);
                }


                for(unsigned int i = 0; i < selectedNodes.size(); i++)
                    nodesColor->InsertValue(selectedNodes[i], 0);

                selectedNodes.clear();


            }
        }
        this->qvtkWidget->update();

        this->InvokeEvent(vtkCommand::StartInteractionEvent);
    }

}

void SemanticGraphInteractionStyle::OnLeftButtonUp()
{
    if(this->Interaction == SELECTING)
      {
        this->Interaction = NONE;

        // Clear the rubber band
        int* size = this->Interactor->GetRenderWindow()->GetSize();
        unsigned char* pixels = this->PixelArray->GetPointer(0);
        this->Interactor->GetRenderWindow()->SetRGBACharPixelData(0, 0, size[0] - 1, size[1] - 1, pixels, 0);
        this->Interactor->GetRenderWindow()->Frame();
        this->Interactor->GetRenderWindow()->Render();

        this->InvokeEvent(vtkCommand::EndInteractionEvent);
      }
}

GraphTemplate::Graph<Annotation *> *SemanticGraphInteractionStyle::getGraph() const
{
    return graph;
}

void SemanticGraphInteractionStyle::setGraph(GraphTemplate::Graph<Annotation *> *value)
{
    graph = value;
}

ExtendedTrimesh *SemanticGraphInteractionStyle::getMesh() const
{
    return mesh;
}

void SemanticGraphInteractionStyle::setMesh(ExtendedTrimesh *value)
{
    mesh = value;
}

vtkSmartPointer<vtkGraphLayoutView> SemanticGraphInteractionStyle::getGraphLayoutView() const
{
    return GraphLayoutView;
}

void SemanticGraphInteractionStyle::setGraphLayoutView(const vtkSmartPointer<vtkGraphLayoutView> &value)
{
    GraphLayoutView = value;
}

QListView *SemanticGraphInteractionStyle::getListView() const
{
    return listView;
}

void SemanticGraphInteractionStyle::setListView(QListView *value)
{
    listView = value;
}

vtkSmartPointer<vtkGraph> SemanticGraphInteractionStyle::getVisualizedGraph() const
{
    return visualizedGraph;
}

void SemanticGraphInteractionStyle::setVisualizedGraph(const vtkSmartPointer<vtkGraph> &value)
{
    visualizedGraph = value;
}

QLabel *SemanticGraphInteractionStyle::getLabel() const
{
    return label;
}

void SemanticGraphInteractionStyle::setLabel(QLabel *value)
{
    label = value;
}

vtkSmartPointer<vtkIntArray> SemanticGraphInteractionStyle::getNodesColor() const
{
    return nodesColor;
}

void SemanticGraphInteractionStyle::setNodesColor(const vtkSmartPointer<vtkIntArray> &value)
{
    nodesColor = value;
}

vtkSmartPointer<vtkIntArray> SemanticGraphInteractionStyle::getEdgesColor() const
{
    return edgesColor;
}

void SemanticGraphInteractionStyle::setEdgesColor(const vtkSmartPointer<vtkIntArray> &value)
{
    edgesColor = value;
}

QVTKOpenGLNativeWidget *SemanticGraphInteractionStyle::getQvtkWidget() const
{
    return qvtkWidget;
}

void SemanticGraphInteractionStyle::setQvtkWidget(QVTKOpenGLNativeWidget *value)
{
    qvtkWidget = value;
}

std::vector<unsigned int> SemanticGraphInteractionStyle::getSelectedNodes() const
{
    return selectedNodes;
}

void SemanticGraphInteractionStyle::setSelectedNodes(const std::vector<unsigned int> &value)
{
    selectedNodes = value;
}

std::vector<unsigned int> SemanticGraphInteractionStyle::getSelectedEdges() const
{
    return selectedEdges;
}

void SemanticGraphInteractionStyle::setSelectedEdges(const std::vector<unsigned int> &value)
{
    selectedEdges = value;
}

vtkSmartPointer<vtkGraphToPolyData> SemanticGraphInteractionStyle::getGraphToPolydataFilter() const
{
    return graphToPolydataFilter;
}

void SemanticGraphInteractionStyle::setGraphToPolydataFilter(const vtkSmartPointer<vtkGraphToPolyData> &value)
{
    graphToPolydataFilter = value;
}
