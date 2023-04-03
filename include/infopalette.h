#ifndef INFOPALETTE_H
#define INFOPALETTE_H

#include <drawablemesh.h>
#include <annotationsconstraint.h>

#include <QWidget>
#include <vector>

namespace Ui {

class InfoPalette;
}

class AnnotationsConstraint;
class InfoPalette : public QWidget
{
    Q_OBJECT

public:

    const int infoHeight = 12;
    explicit InfoPalette(QWidget *parent = nullptr);
    ~InfoPalette();

    void updateInfo();


    std::vector<AnnotationsConstraint *> getConstraints() const;
    void setConstraints(const std::vector<AnnotationsConstraint *> &value);

    DrawableMesh *getMesh() const;
    void setMesh(DrawableMesh *value);
private slots:
    void currentIndexChangedSlot(int);

private:
    std::vector<AnnotationsConstraint*> constraints;
    DrawableMesh* mesh;
    Ui::InfoPalette *ui;
};

#endif // INFOPALETTE_H
