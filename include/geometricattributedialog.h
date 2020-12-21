#ifndef GEOMETRICATTRIBUTEDIALOG_H
#define GEOMETRICATTRIBUTEDIALOG_H

#include <QDialog>
#include <drawablemesh.h>
#include <vtkSmartPointer.h>
#include <measurestyle.h>
#include <annotationselectioninteractorstyle.h>

namespace Ui {
class GeometricAttributeDialog;
}

class GeometricAttributeDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GeometricAttributeDialog(QWidget *parent = nullptr);
    ~GeometricAttributeDialog();

    void updateView();

    DrawableMesh *getMesh() const;
    void setMesh(DrawableMesh *value);

public slots:
    void rulerSelected(bool);
    void tapeSelected(bool);
    void caliperSelected(bool);

private slots:
    void on_addMeasureButton_clicked();

private:
    Ui::GeometricAttributeDialog *ui;
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkPropAssembly> canvas;
    vtkSmartPointer<MeasureStyle> measureStyle;
    DrawableMesh* mesh;
};

#endif // GEOMETRICATTRIBUTEDIALOG_H
