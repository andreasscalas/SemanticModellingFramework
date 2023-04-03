#ifndef TRANSPARENCYDIALOG_H
#define TRANSPARENCYDIALOG_H

#include "drawablemesh.h"

#include <QDialog>

namespace Ui {
class TransparencyDialog;
}

class TransparencyDialog : public QDialog
{
    Q_OBJECT

public:
    explicit TransparencyDialog(QWidget *parent = nullptr);
    ~TransparencyDialog();
    DrawableMesh *getMesh() const;
    void setMesh(DrawableMesh *value);
signals:
    void updateView(DrawableMesh*);

private slots:
    void on_buttonBox_rejected();

private slots:
    void on_opacitySlider_valueChanged(int value);
    void on_buttonBox_accepted();

private:
    DrawableMesh* mesh;
    double originalOpacity;
    Ui::TransparencyDialog *ui;
};

#endif // TRANSPARENCYDIALOG_H
