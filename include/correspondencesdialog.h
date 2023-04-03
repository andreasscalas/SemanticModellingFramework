#ifndef CORRESPONDENCESDIALOG_H
#define CORRESPONDENCESDIALOG_H

#include <QDialog>
#include <drawablemesh.h>
#include <correspondencesselectionstyle.h>

namespace Ui {
class CorrespondencesDialog;
}

class CorrespondencesDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CorrespondencesDialog(QWidget *parent = nullptr);
    ~CorrespondencesDialog();

    DrawableMesh *getTemplateMesh() const;
    void setTemplateMesh(DrawableMesh *value);

    DrawableMesh *getFragmentMesh() const;
    void setFragmentMesh(DrawableMesh *value);

    void updateView();

    std::vector<std::pair<int, int> > getCorrespondences() const;

private slots:
    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

private:
    Ui::CorrespondencesDialog *ui;
    DrawableMesh* templateMesh;
    DrawableMesh* fragmentMesh;
    std::vector<std::pair<int, int> > correspondences;
    vtkSmartPointer<CorrespondencesSelectionStyle> templateSelectionStyle;
    vtkSmartPointer<CorrespondencesSelectionStyle> fragmentSelectionStyle;
    vtkSmartPointer<vtkRenderer> templateRenderer;
    vtkSmartPointer<vtkRenderer> fragmentRenderer;
    vtkSmartPointer<vtkPropAssembly> originalTemplateCanvas;
    vtkSmartPointer<vtkPropAssembly> originalFragmentCanvas;
};

#endif // CORRESPONDENCESDIALOG_H
