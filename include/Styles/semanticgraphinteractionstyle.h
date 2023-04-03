#ifndef SEMANTICGRAPHINTERACTIONSTYLE_H
#define SEMANTICGRAPHINTERACTIONSTYLE_H

#include <vtkInteractorStyleRubberBand2D.h>
#include <vtkSmartPointer.h>
#include <vtkPointPicker.h>
#include <vtkActor.h>
#include <vtkGraph.h>
#include <vtkGraphLayoutView.h>
#include <vtkGraphToPolyData.h>
#include <vtkCellPicker.h>
#include <QStringListModel>
#include <QLabel>
#include <QListView>
#include <QVTKOpenGLNativeWidget.h>

#include <DataStructures/include/graph.h>
#include <annotation.h>
#include <extendedtrimesh.h>

#define VTKISRBP_ORIENT 0
#define VTKISRBP_SELECT 1

class SemanticGraphInteractionStyle : public vtkInteractorStyleRubberBand2D
{
public:
    SemanticGraphInteractionStyle();

    static SemanticGraphInteractionStyle* New();
    vtkTypeMacro(SemanticGraphInteractionStyle, vtkInteractorStyleRubberBand2D)

    void OnRightButtonDown() override;
    void OnRightButtonUp() override;
    void OnMouseMove() override;
    void OnLeftButtonDown() override;
    void OnLeftButtonUp() override;

    GraphTemplate::Graph<Annotation *> *getGraph() const;
    void setGraph(GraphTemplate::Graph<Annotation *> *value);

    ExtendedTrimesh *getMesh() const;
    void setMesh(ExtendedTrimesh *value);

    vtkSmartPointer<vtkGraphLayoutView> getGraphLayoutView() const;
    void setGraphLayoutView(const vtkSmartPointer<vtkGraphLayoutView> &value);

    QListView *getListView() const;
    void setListView(QListView *value);

    vtkSmartPointer<vtkGraph> getVisualizedGraph() const;
    void setVisualizedGraph(const vtkSmartPointer<vtkGraph> &value);

    QLabel *getLabel() const;
    void setLabel(QLabel *value);

    vtkSmartPointer<vtkIntArray> getNodesColor() const;
    void setNodesColor(const vtkSmartPointer<vtkIntArray> &value);

    vtkSmartPointer<vtkIntArray> getEdgesColor() const;
    void setEdgesColor(const vtkSmartPointer<vtkIntArray> &value);

    QVTKOpenGLNativeWidget *getQvtkWidget() const;
    void setQvtkWidget(QVTKOpenGLNativeWidget *value);

    std::vector<unsigned int> getSelectedNodes() const;
    void setSelectedNodes(const std::vector<unsigned int> &value);

    std::vector<unsigned int> getSelectedEdges() const;
    void setSelectedEdges(const std::vector<unsigned int> &value);

    vtkSmartPointer<vtkGraphToPolyData> getGraphToPolydataFilter() const;
    void setGraphToPolydataFilter(const vtkSmartPointer<vtkGraphToPolyData> &value);

protected:
    GraphTemplate::Graph<Annotation *> *graph;
    ExtendedTrimesh *mesh;
    std::vector<unsigned int> selectedNodes;
    std::vector<unsigned int> selectedEdges;
    std::map<GraphTemplate::Node<Annotation*>*, unsigned int> nodesIds;

    QStringListModel *model;
    QListView *listView;
    QLabel *label;
    QVTKOpenGLNativeWidget *qvtkWidget;

    vtkSmartPointer<vtkGraph> visualizedGraph;
    vtkSmartPointer<vtkGraphToPolyData> graphToPolydataFilter;
    vtkSmartPointer<vtkIntArray> nodesColor;
    vtkSmartPointer<vtkIntArray> edgesColor;

    vtkSmartPointer<vtkPointPicker> PointPicker;
    vtkSmartPointer<vtkCellPicker> CellPicker;
    vtkSmartPointer<vtkGraphLayoutView> GraphLayoutView;
};

#endif // SEMANTICGRAPHINTERACTIONSTYLE_H
