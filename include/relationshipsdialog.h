#ifndef RELATIONSHIPSDIALOG_H
#define RELATIONSHIPSDIALOG_H

#include "annotationconstraintdialog.h"

#include <QMainWindow>


#include <annotation.h>
#include <semanticgraphinteractionstyle.h>
#include <drawablegraph.h>
#include <annotationsrelationship.h>

#include <vtkMutableDirectedGraph.h>

namespace Ui {
class RelationshipsDialog;
}

class RelationshipsDialog : public QMainWindow
{
    Q_OBJECT

public:
    explicit RelationshipsDialog(QWidget *parent = nullptr);
    ~RelationshipsDialog();

    void update();
    void updateView();

    ExtendedTrimesh *getMesh() const;
    void setMesh(ExtendedTrimesh *value);
    bool getIsSystemConstrainable() const;
    void setIsSystemConstrainable(bool value);

    bool getIsSystemConstrained() const;
    void setIsSystemConstrained(bool value);

public slots:
    void slotSave();
    void slotLoad();
    void slotAddRelationship();
    void slotAddAnnotationsRelationship(std::string, double, double, double,  unsigned int, unsigned int, bool);
    void slotConstrainRelationship();
    void slotReleaseConstraints();
signals:
    void constrainRelationship(AnnotationsRelationship*&);
    void constrainRelationships();
    void releaseConstraints();
    void addSemanticRelationship(std::vector<Annotation*>, std::string, double, double, double, unsigned int, unsigned int, bool);

private:
    Ui::RelationshipsDialog *ui;
    ExtendedTrimesh* mesh;
    vtkSmartPointer<SemanticGraphInteractionStyle> interactorStyle;
    vtkSmartPointer<vtkMutableDirectedGraph> g;
    AnnotationConstraintDialog *cd ;
    GraphTemplate::DrawableGraph<Annotation*>* drawableGraph;

    bool isSystemConstrainable;
    bool isSystemConstrained;
};

#endif // RELATIONSHIPSDIALOG_H
