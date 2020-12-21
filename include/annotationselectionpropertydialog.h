#ifndef ANNOTATIONSELECTIONPROPERTYDIALOG_H
#define ANNOTATIONSELECTIONPROPERTYDIALOG_H

#include <QDialog>
#include <annotationselectioninteractorstyle.h>

namespace Ui {
class AnnotationSelectionPropertyDialog;
}

class AnnotationSelectionPropertyDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AnnotationSelectionPropertyDialog(QWidget *parent = nullptr);
    ~AnnotationSelectionPropertyDialog();

    void updateView();

    DrawableMesh *getMesh() const;
    void setMesh(DrawableMesh *value);

private slots:
    void on_buttonBox_accepted();
    void on_buttonBox_rejected();

private:
    Ui::AnnotationSelectionPropertyDialog *ui;
    vtkSmartPointer<AnnotationSelectionInteractorStyle> annotationStyle;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkPropAssembly> canvas;
    DrawableMesh* mesh;
    std::map<unsigned int, bool> annotationsSelectionStatus;
};

#endif // ANNOTATIONSELECTIONPROPERTYDIALOG_H
