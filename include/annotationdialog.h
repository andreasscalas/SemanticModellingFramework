#ifndef ANNOTATIONDIALOG_H
#define ANNOTATIONDIALOG_H

#include <QDialog>
#include <string>

namespace Ui {
    class AnnotationDialog;
}

class AnnotationDialog : public QDialog{
    Q_OBJECT

    public:
        AnnotationDialog(QWidget* parent);
        ~AnnotationDialog();
    signals:
        void finalizationCalled(std::string, uchar*);

    private slots:
        void slotColor();
        void slotOntologyDialog();
        void slotTagEdit(QString);
        void slotSave();


    private:
        Ui::AnnotationDialog *ui;
        unsigned char color[3];
        std::string tag;
};

#endif // ANNOTATIONDIALOG_H
