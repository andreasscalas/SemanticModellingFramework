#ifndef ANNOTATIONSELECTIONDIALOG_H
#define ANNOTATIONSELECTIONDIALOG_H

#include <QDialog>
#include <drawableannotation.h>
#include <vector>
namespace Ui {
class AnnotationSelectionDialog;
}

class AnnotationSelectionDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AnnotationSelectionDialog(QWidget *parent = nullptr);
    ~AnnotationSelectionDialog();

    std::vector<DrawableAnnotation *> getAnnotationsList() const;
    void setAnnotationsList(const std::vector<DrawableAnnotation *> &value);

    DrawableAnnotation *getSelectedAnnotation() const;

private slots:
    void on_selectAnnotationButton_clicked();

private:
    Ui::AnnotationSelectionDialog *ui;
    std::vector<DrawableAnnotation*> annotationsList;
    DrawableAnnotation* selectedAnnotation;
};

#endif // ANNOTATIONSELECTIONDIALOG_H
