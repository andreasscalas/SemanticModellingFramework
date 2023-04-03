#ifndef COLORFORM_H
#define COLORFORM_H

#include <QDialog>
#include <QColor>
#include "ui_colorform.h"
class ColorForm : public QDialog, public Ui::ColorForm{

    Q_OBJECT

    public:
        ColorForm(QWidget *parent = nullptr);

    private slots:
        void on_meshButton_clicked();
        void on_cageButton_clicked();

    signals:
        void meshColorChanged(QColor meshColor);
        void cageColorChanged(QColor cageColor);
    private:
        QColor meshColor, cageColor;

};

#endif // COLORFORM_H
