#ifndef CONSTRAINTDIALOG_H
#define CONSTRAINTDIALOG_H

#include <QDialog>
#include <QFileDialog>
#include <QListWidget>
#include <QListWidgetItem>
#include <string>
#include <utilities.h>

namespace Ui {
    class ConstraintDialog;
}

class ConstraintDialog : public QDialog{
    Q_OBJECT

    public:
        ConstraintDialog(QWidget* parent);
        ~ConstraintDialog();
    signals:
        void addConstraint(Utilities::ConstraintType, double);

    private slots:
        void slotType(int);
        void slotWeight(int);
        void slotRelative(int);
        void slotCancel();
        void slotSave();


    private:
        Ui::ConstraintDialog *ui;
        Utilities::ConstraintType type;
        double weight;
        bool relative;
};

#endif // CONSTRAINTDIALOG_H
