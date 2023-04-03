#ifndef LAYERDIALOG_H
#define LAYERDIALOG_H

#include <map>

#include <drawablemesh.h>

#include <QWidget>
#include <QCheckBox>

namespace Ui {
class LayerDialog;
}

class LayerDialog : public QWidget
{
    Q_OBJECT

public:
    explicit LayerDialog(QWidget *parent = nullptr);
    ~LayerDialog();
    void addFragment(std::string, DrawableMesh* mesh);
    void addFragment(QString, DrawableMesh* mesh);
    void removeFragment(DrawableMesh* mesh);
    void addMainMesh(std::string, DrawableMesh* mesh);
    void addMainMesh(QString, DrawableMesh* mesh);
    void removeMainMesh(DrawableMesh* mesh);
    void addCage(std::string, DrawableMesh* mesh);
    void addCage(QString, DrawableMesh* mesh);
    void removeCage(DrawableMesh* mesh);
    bool getConstraintsImposed() const;
    void setConstraintsImposed(bool value);

    bool getBcComputed() const;
    void setBcComputed(bool value);

signals:
    void updateMeshView(DrawableMesh*);
    void deleteMesh(DrawableMesh*);
    void editAnnotations(DrawableMesh*);
    void fitRigidly(DrawableMesh*);
    void adaptTemplate(DrawableMesh*);
    void openAnnotations(DrawableMesh*);
    void removeAnnotations(DrawableMesh*);
    void openCage(DrawableMesh*);
    void deleteCage(DrawableMesh*);

private:
    Ui::LayerDialog *ui;
    std::map<QCheckBox*, DrawableMesh*> checkbox_mesh_correspondence;
    bool constraintsImposed;
    bool bcComputed;
private slots:
    void slotStateChanged(int);
    void showContextMenu(const QPoint&);
    void slotUpdateView(DrawableMesh*);
};

#endif // LAYERDIALOG_H
