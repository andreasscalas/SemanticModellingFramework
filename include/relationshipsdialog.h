#ifndef RELATIONSHIPSDIALOG_H
#define RELATIONSHIPSDIALOG_H

#include "annotationconstraintdialog.h"

#include <QMainWindow>
#include <annotation.h>
#include <semanticgraphinteractionstyle.h>
#include <vtkMutableDirectedGraph.h>
#include <DataStructures/include/graph.h>


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

public slots:
    void slotSave();
    void slotLoad();
    void slotAddRelationship();
    void slotAddAnnotationsRelationship(std::string, double, double, double,  unsigned int, unsigned int, bool);
    void slotConstrainRelationship();
signals:
    void constrainRelationship(AnnotationsRelationship*&);
private:
    Ui::RelationshipsDialog *ui;
    ExtendedTrimesh* mesh;
    vtkSmartPointer<SemanticGraphInteractionStyle> interactorStyle;
    vtkSmartPointer<vtkMutableDirectedGraph> g;
    AnnotationConstraintDialog *cd ;
};

#endif // RELATIONSHIPSDIALOG_H
