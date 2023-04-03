#include "relationshipsdialog.h"
#include "ui_relationshipsdialog.h"

#include <QFileDialog>

#include <vtkStringArray.h>
#include <vtkGraphLayoutView.h>
#include <vtkDataSetAttributes.h>
#include <vtkGraphLayout.h>
#include <vtkgraphtopolydataextended.h>
#include <vtkGlyphSource2D.h>
#include <vtkGlyph3D.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkLookupTable.h>
#include <vtkViewTheme.h>
#include <vtkFloatArray.h>
#include <vtkForceDirectedLayoutStrategy.h>
#include <vtkRenderedAreaPicker.h>
#include <vtkPolyDataMapper.h>
//#include <vtkRenderedGraphRepresentation.h>
//#include <vtkGraphToGlyphs.h>

vtkStandardNewMacro(vtkGraphToPolydataExtended)

vtkStandardNewMacro(SemanticGraphInteractionStyle)

RelationshipsDialog::RelationshipsDialog(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::RelationshipsDialog)
{
    ui->setupUi(this);
    this->ui->actionConstrainRelationship->setEnabled(false);
    this->ui->actionReleaseConstraints->setEnabled(false);
    interactorStyle = vtkSmartPointer<SemanticGraphInteractionStyle>::New();
    g = vtkSmartPointer<vtkMutableDirectedGraph>::New();
    connect(this->ui->actionSave, SIGNAL(triggered()), this, SLOT(slotSave()));
    connect(this->ui->actionLoad, SIGNAL(triggered()), this, SLOT(slotLoad()));
    connect(this->ui->actionAddRelationship, SIGNAL(triggered()), this, SLOT(slotAddRelationship()));
    connect(this->ui->actionConstrainRelationship, SIGNAL(triggered()), this, SLOT(slotConstrainRelationship()));
    connect(this->ui->actionReleaseConstraints, SIGNAL(triggered()), this, SLOT(slotReleaseConstraints()));
}

RelationshipsDialog::~RelationshipsDialog()
{
    delete ui;
}

void RelationshipsDialog::update()
{
    g = vtkSmartPointer<vtkMutableDirectedGraph>::New();
    std::map<GraphTemplate::Node<Annotation*>*, unsigned int> map;
    vtkSmartPointer<vtkStringArray> annotationsTag = vtkSmartPointer<vtkStringArray>::New();
    vtkSmartPointer<vtkStringArray> relType = vtkSmartPointer<vtkStringArray>::New();
    vtkSmartPointer<vtkFloatArray> nodesScales = vtkSmartPointer<vtkFloatArray>::New();
    vtkSmartPointer<vtkIntArray> nodesColor = vtkSmartPointer<vtkIntArray>::New();
    vtkSmartPointer<vtkIntArray> edgesColor = vtkSmartPointer<vtkIntArray>::New();
    vtkSmartPointer<vtkFloatArray> edgesGlyphsPositions = vtkSmartPointer<vtkFloatArray>::New();

    annotationsTag->SetNumberOfComponents(1);
    annotationsTag->SetName("AnnotationsTag");
    relType->SetNumberOfComponents(1);
    relType->SetName("RelType");

    nodesScales->SetNumberOfComponents(1);
    nodesScales->SetName("NodesScales");
    nodesColor->SetNumberOfComponents(1);
    nodesColor->SetName("NodesColor");
    edgesColor->SetNumberOfComponents(1);
    edgesColor->SetName("EdgesColor");
    edgesGlyphsPositions->SetNumberOfComponents(1);
    edgesGlyphsPositions->SetName("EdgeGlyphsPositions");

    for(unsigned int i = 0; i < mesh->getGraph()->getNodes().size(); i++){
        map.insert(std::make_pair(mesh->getGraph()->getNodes()[i], i));
        nodesScales->InsertNextValue(5.0);
        nodesColor->InsertNextValue(0);
        g->AddVertex();
        annotationsTag->InsertNextValue(mesh->getGraph()->getNodes()[i]->getData()->getTag());
    }

    g->GetVertexData()->AddArray(annotationsTag);
    g->GetVertexData()->AddArray(nodesScales);
    g->GetVertexData()->AddArray(nodesColor);

    for(unsigned int i = 0 ; i < mesh->getGraph()->getArcs().size(); i++){
        g->AddEdge(map[mesh->getGraph()->getArcs()[i]->getN1()], map[mesh->getGraph()->getArcs()[i]->getN2()]);
        edgesColor->InsertNextValue(0);
        if(mesh->getGraph()->getArcs()[i]->isDirected())
            edgesGlyphsPositions->InsertNextValue(1.0);
        else
            edgesGlyphsPositions->InsertNextValue(0.5);
        std::string label(mesh->getGraph()->getArcs()[i]->getLabel());
        relType->InsertNextValue(label + ": " + std::to_string(mesh->getGraph()->getArcs()[i]->getWeight()));
    }

    g->GetEdgeData()->AddArray(relType);
    g->GetEdgeData()->AddArray(edgesGlyphsPositions);
    g->GetEdgeData()->AddArray(edgesColor);
    interactorStyle = vtkSmartPointer<SemanticGraphInteractionStyle>::New();
    interactorStyle->setNodesColor(nodesColor);
    interactorStyle->setEdgesColor(edgesColor);
    interactorStyle->setVisualizedGraph(g);
    interactorStyle->setGraph(this->mesh->getGraph());
    interactorStyle->setListView(this->ui->listView);
    interactorStyle->setLabel(this->ui->label);
    interactorStyle->setQvtkWidget(this->ui->qvtkWidget);

//    drawableGraph = new GraphTemplate::DrawableGraph<Annotation*>(mesh->getGraph());
//    drawableGraph->setFrameLength(100);
//    drawableGraph->setFrameWidth(100);
//    drawableGraph->update();
//    drawableGraph->updateView();

}

void RelationshipsDialog::updateView()
{
    vtkSmartPointer<vtkForceDirectedLayoutStrategy> strategy = vtkSmartPointer<vtkForceDirectedLayoutStrategy>::New();
    vtkSmartPointer<vtkGraphLayoutView> graphLayoutView = vtkSmartPointer<vtkGraphLayoutView>::New();
    vtkSmartPointer<vtkGraphLayout> layout =  vtkSmartPointer<vtkGraphLayout>::New();
    vtkSmartPointer<vtkGraphToPolydataExtended> graphToPolydataFilter = vtkSmartPointer<vtkGraphToPolydataExtended>::New();
    vtkSmartPointer<vtkGlyphSource2D> arrowSource = vtkSmartPointer<vtkGlyphSource2D>::New();
    vtkSmartPointer<vtkGlyph3D> arrowGlyph = vtkSmartPointer<vtkGlyph3D>::New();
    vtkSmartPointer<vtkPolyDataMapper> arrowMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkSmartPointer<vtkActor> arrowActor = vtkSmartPointer<vtkActor>::New();
    vtkSmartPointer<vtkPolyDataMapper> glyphsMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkSmartPointer<vtkActor> glyphsActor = vtkSmartPointer<vtkActor>::New();

    vtkSmartPointer<vtkLookupTable> lookupTable = vtkSmartPointer<vtkLookupTable>::New();
    lookupTable->SetNumberOfTableValues(2);
    lookupTable->SetTableValue(0, 1.0, 1.0, 1.0); // white
    lookupTable->SetTableValue(1, 1.0, 0.0, 0.0); // red
    lookupTable->Build();

    layout->SetInputData(g);
    //strategy->ThreeDimensionalLayoutOn();
    layout->SetLayoutStrategy(strategy);
    layout->Update();

    graphLayoutView->SetVertexColorArrayName("NodesColor");
    graphLayoutView->SetEdgeColorArrayName("EdgesColor");
    vtkSmartPointer<vtkViewTheme> theme = vtkSmartPointer<vtkViewTheme>::New();
    theme->SetPointLookupTable(lookupTable);
    theme->SetCellLookupTable(lookupTable);
    graphLayoutView->ApplyViewTheme(theme);
    graphLayoutView->ColorVerticesOn();
    graphLayoutView->ColorEdgesOn();

    graphToPolydataFilter->SetInputConnection(layout->GetOutputPort());
    graphToPolydataFilter->setEdgeGlyphPositions(static_cast<vtkFloatArray*>(g->GetEdgeData()->GetArray("EdgeGlyphsPositions")));
    graphToPolydataFilter->EdgeGlyphOutputOn();

    // Set the position (0: edge start, 1: edge end) where
    // the edge arrows should go.
    /*
    graphToPolydataFilter->SetEdgeGlyphPosition(0);*/

    // Make a simple edge arrow for glyphing.
    arrowSource->SetGlyphTypeToEdgeArrow();
    arrowSource->SetScale(0.03);
    arrowSource->Update();

    // Use Glyph3D to repeat the glyph on all edges.
    arrowGlyph->SetInputConnection(0, graphToPolydataFilter->GetOutputPort(1));
    arrowGlyph->SetInputConnection(1, arrowSource->GetOutputPort());

    // Add the edge arrow actor to the view.
    arrowMapper->SetInputConnection(arrowGlyph->GetOutputPort());
    arrowActor->SetMapper(arrowMapper);
    graphLayoutView->GetRenderer()->AddActor(arrowActor);

    graphLayoutView->SetLayoutStrategy(strategy);
    graphLayoutView->AddRepresentationFromInputConnection(layout->GetOutputPort());
    graphLayoutView->SetVertexLabelVisibility(true);
    graphLayoutView->SetEdgeLabelVisibility(true);
    graphLayoutView->SetEdgeLabelArrayName("RelType"); //default is "labels"
    graphLayoutView->SetVertexLabelArrayName("AnnotationsTag"); //default is "labels"
    graphLayoutView->ScaledGlyphsOn();
    graphLayoutView->SetScalingArrayName("NodesScales");
    graphLayoutView->Modified();
    graphLayoutView->Update();
    graphLayoutView->ResetCamera();
    graphLayoutView->GetRenderer()->AddActor(arrowActor);
//    vtkRenderedGraphRepresentation::SafeDownCast(graphLayoutView->GetRepresentation())
//              ->SetGlyphType(vtkGraphToGlyphs::CIRCLE);
    graphLayoutView->SetInteractor(this->ui->qvtkWidget->interactor());
    interactorStyle->setGraphLayoutView(graphLayoutView);

    vtkSmartPointer<vtkRenderedAreaPicker> AreaPicker = vtkSmartPointer<vtkRenderedAreaPicker>::New();
    interactorStyle->SetInteractor(graphLayoutView->GetRenderWindow()->GetInteractor());
    interactorStyle->setGraphToPolydataFilter(graphToPolydataFilter);
    interactorStyle->GetInteractor()->SetPicker(AreaPicker);
    graphLayoutView->SetInteractorStyle(interactorStyle);
    this->ui->qvtkWidget->setRenderWindow(graphLayoutView->GetRenderWindow());
    //vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    //vtkSmartPointer<vtkPropAssembly> assembly = vtkSmartPointer<vtkPropAssembly>::New();
    //drawableGraph->draw(assembly);
    //renderer->AddActor(assembly);
    //this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);
    this->ui->qvtkWidget->update();
}


void RelationshipsDialog::slotAddRelationship()
{
    std::vector<unsigned int> selectedNodes = interactorStyle->getSelectedNodes();

    if(selectedNodes.size() < 2)
        return;    

    std::vector<GraphTemplate::Node<Annotation*> *> nodes = mesh->getGraph()->getNodes();
    std::vector<Annotation*> selectedAnnotations;

    for(unsigned int i = 0; i < selectedNodes.size(); i++)
        selectedAnnotations.push_back(nodes[selectedNodes[i]]->getData());

    cd = new AnnotationConstraintDialog(this);

    cd->setSubjects(selectedAnnotations);
    cd->show();

    connect(cd, SIGNAL(addSemanticRelationship(std::string, double, double, double, unsigned int, unsigned int, bool)), this, SLOT(slotAddAnnotationsRelationship(std::string, double, double, double, unsigned int, unsigned int, bool)));


}

void RelationshipsDialog::slotAddAnnotationsRelationship(std::string type, double weight, double minValue, double maxValue, unsigned int measure1ID, unsigned int measure2ID, bool directed)
{

    emit(addSemanticRelationship(cd->getSubjects(), type, weight, minValue, maxValue, measure1ID, measure2ID, directed));
    this->update();
    this->updateView();

}

void RelationshipsDialog::slotConstrainRelationship()
{
    /*std::vector<unsigned int> selectedEdges = interactorStyle->getSelectedEdges();
    std::vector<GraphTemplate::Arc<Annotation*> *> arcs = mesh->getGraph()->getArcs();
    std::vector<AnnotationsRelationship*> relationships;
    std::vector<Annotation*> involvedAnnotations;
    std::string type;
    for(unsigned int i = 0; i < selectedEdges.size(); i++){
        GraphTemplate::Arc<Annotation*> *arc = arcs[selectedEdges[i]];
        if(arc->getInfo() == nullptr) exit(12);
        AnnotationsRelationship* relationship = static_cast<AnnotationsRelationship*>(arc->getInfo());
        std::vector<AnnotationsRelationship*>::iterator it1 = std::find(relationships.begin(), relationships.end(), relationship);
        if(it1 == relationships.end())
            relationships.push_back(relationship);
    }

    if(relationships.size() != 1) exit(12);

    emit constrainRelationship(relationships[0]);
    */


    emit constrainRelationships();
    setIsSystemConstrained(true);
}

void RelationshipsDialog::slotReleaseConstraints()
{
    emit releaseConstraints();
    setIsSystemConstrained(false);
}

bool RelationshipsDialog::getIsSystemConstrainable() const
{
    return isSystemConstrainable;
}

void RelationshipsDialog::setIsSystemConstrainable(bool value)
{
    isSystemConstrainable = value;
    this->ui->actionConstrainRelationship->setEnabled(value);
}

bool RelationshipsDialog::getIsSystemConstrained() const
{
    return isSystemConstrained;
}

void RelationshipsDialog::setIsSystemConstrained(bool value)
{
    isSystemConstrained = value;
    this->ui->actionReleaseConstraints->setEnabled(value);
    setIsSystemConstrainable(!value);
}

ExtendedTrimesh *RelationshipsDialog::getMesh() const
{
    return mesh;
}

void RelationshipsDialog::setMesh(ExtendedTrimesh *value)
{
    mesh = value;
}

void RelationshipsDialog::slotSave()
{
    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save the annotations on the mesh",
                       ".",
                       "ANT(*.ant);;FCT(*.fct);;TRIANT(*.triant);;");

    if (!filename.isEmpty()){
        GraphTemplate::Graph<Annotation*>* graph = mesh->getGraph();
        std::ofstream relationshipsFile;
        rapidjson::StringBuffer s;
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(s);
        writer.StartObject();
        writer.Key("relationships");
        writer.StartArray();
        for(unsigned int i = 0; i < graph->getArcs().size(); i++){
            GraphTemplate::Arc<Annotation*>* a = graph->getArcs()[i];
            writer.Key("id");
            writer.Uint(a->getId());
            writer.Key("n1");
            writer.Uint(a->getN1()->getData()->getId());
            writer.Key("n2");
            writer.Uint(a->getN2()->getData()->getId());
            writer.Key("label");
            writer.String(a->getLabel().c_str());
            writer.Key("weight");
            writer.Double(a->getWeight());

        }


    }
}

void RelationshipsDialog::slotLoad()
{

}
