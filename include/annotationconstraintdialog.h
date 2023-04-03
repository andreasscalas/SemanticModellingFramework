#ifndef ANNOTATIONCONSTRAINTDIALOG_H
#define ANNOTATIONCONSTRAINTDIALOG_H

#include <QDialog>
#include <QDoubleSpinBox>
#include <string>
#include <annotation.h>

namespace Ui {
    class AnnotationConstraintDialog;
}

class AnnotationConstraintDialog : public QDialog{
    Q_OBJECT

    public:
        AnnotationConstraintDialog (QWidget* parent);
        ~AnnotationConstraintDialog ();
        std::vector<Annotation *> getSubjects() const;
        void setSubjects(const std::vector<Annotation *> &value);

        std::string getTypeString() const;

signals:
        void addSemanticConstraint(std::string, double, double, double, unsigned int, unsigned int, bool);
        void addSemanticRelationship(std::string, double, double, double, unsigned int, unsigned int, bool);

    private slots:
        void slotType(QString);
        void slotSave();


    private:
        Ui::AnnotationConstraintDialog *ui;
        std::string type;
        std::vector<Annotation*> subjects;
        bool directed;
};

#endif // ANNOTATIONCONSTRAINTDIALOG_H
