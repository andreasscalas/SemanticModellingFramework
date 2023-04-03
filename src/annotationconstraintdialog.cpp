#include "annotationconstraintdialog.h"
#include "ui_annotationconstraintdialog.h"

AnnotationConstraintDialog::AnnotationConstraintDialog(QWidget *parent) : QDialog(parent), ui(new Ui::AnnotationConstraintDialog){
    ui->setupUi(this);
    this->ui->doubleSpinBox1->setEnabled(false);
    this->ui->doubleSpinBox2->setEnabled(false);
    connect(this->ui->typeList, SIGNAL(currentTextChanged(QString)), this, SLOT(slotType(QString)));
    connect(this->ui->SaveButton, SIGNAL(clicked()), this, SLOT(slotSave()));
}

AnnotationConstraintDialog::~AnnotationConstraintDialog(){
    delete ui;
}

void AnnotationConstraintDialog::slotType(QString type)
{
    this->type = type.toStdString();
    if(this->type.compare("") == 0) {
        return;
    } else if(this->type.compare("Surface area") == 0) {
        this->ui->doubleSpinBox1->setValue(0.0);
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setValue(9999.9);
        this->ui->doubleSpinBox2->setEnabled(true);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Surface planarity") == 0) {
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Surface rigidity") == 0) {
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Surface similarity") == 0) {
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Surface curvature") == 0) {
        this->ui->doubleSpinBox1->setValue(0.0);
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setValue(9999.9);
        this->ui->doubleSpinBox2->setEnabled(true);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Surface measure") == 0) {
        this->ui->doubleSpinBox1->setValue(0.0);
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setValue(9999.9);
        this->ui->doubleSpinBox2->setEnabled(true);
        this->ui->measure1ComboBox->setEnabled(true);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Surface same measure") == 0) {
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setEnabled(true);
        this->ui->measure1ComboBox->setEnabled(true);
        this->ui->measure2ComboBox->setEnabled(true);
    } else if(this->type.compare("Surface orientation") == 0) {
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Surface regression plane orientation") == 0) {
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Surface keeping") == 0) {
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Line linearity") == 0) {
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Line planarity") == 0) {
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Line circularity") == 0) {
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Line rectangularity") == 0) {
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Line angle") == 0) {
        this->ui->doubleSpinBox1->setValue(0.0);
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setValue(M_PI);
        this->ui->doubleSpinBox2->setEnabled(true);
    } else if(this->type.compare("Line length") == 0) {
        this->ui->doubleSpinBox1->setValue(0.0);
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setValue(9999.9);
        this->ui->doubleSpinBox2->setEnabled(true);
        this->ui->measure1ComboBox->setEnabled(true);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Line safeguard") == 0) {
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Line keeping") == 0) {
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Point closeness") == 0) {
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Point laplacian") == 0) {
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Point laplacian displacement") == 0) {
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Surfaces co-planarity") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    }  else if(this->type.compare("Surfaces structural continuity") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Surfaces same area") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(true);
        this->ui->measure2ComboBox->setEnabled(true);
    } else if(this->type.compare("Surfaces rigidity") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
    } else if(this->type.compare("Surfaces same measure") == 0) {
        this->directed = true;
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setEnabled(true);
        this->ui->measure1ComboBox->setEnabled(true);
        this->ui->measure2ComboBox->setEnabled(true);
    } else if(this->type.compare("Surfaces same width") == 0) {
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setEnabled(true);
        this->ui->measure1ComboBox->setEnabled(true);
        this->ui->measure2ComboBox->setEnabled(true);
    } else if(this->type.compare("Surfaces same orientation") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Surfaces angle") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setValue(0.0);
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setValue(M_PI);
        this->ui->doubleSpinBox2->setEnabled(true);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Surfaces same level") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);


    } else if(this->type.compare("Surfaces symmetry") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Lines parallelism") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Lines perpendicularity") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Lines same length") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setValue(1.0);
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setValue(1.0);
        this->ui->doubleSpinBox2->setEnabled(true);
        this->ui->measure1ComboBox->setEnabled(true);
        this->ui->measure2ComboBox->setEnabled(true);
    } else if(this->type.compare("Lines symmetry") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Lines coplanarity") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Lines same level") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Points edge strain") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setValue(0.0);
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setValue(9999.9);
        this->ui->doubleSpinBox2->setEnabled(true);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Points triangle strain") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setValue(0.0);
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setValue(9999.9);
        this->ui->doubleSpinBox2->setEnabled(true);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Points tetrahedron strain") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setValue(0.0);
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setValue(9999.9);
        this->ui->doubleSpinBox2->setEnabled(true);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Points area") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setValue(0.0);
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setValue(9999.9);
        this->ui->doubleSpinBox2->setEnabled(true);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Points volume") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setValue(0.0);
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setValue(9999.9);
        this->ui->doubleSpinBox2->setEnabled(true);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Points bending") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setValue(0.0);
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setValue(9999.9);
        this->ui->doubleSpinBox2->setEnabled(true);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Points line") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Points plane") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Points circle") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Points sphere") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Points rigidity") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Points similarity") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Points rectangle") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Points parallelogram") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setEnabled(false);
        this->ui->doubleSpinBox2->setEnabled(false);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else if(this->type.compare("Points angle") == 0) {
        this->directed = false;
        this->ui->doubleSpinBox1->setValue(0.0);
        this->ui->doubleSpinBox1->setEnabled(true);
        this->ui->doubleSpinBox2->setValue(M_PI);
        this->ui->doubleSpinBox2->setEnabled(true);
        this->ui->measure1ComboBox->setEnabled(false);
        this->ui->measure2ComboBox->setEnabled(false);
    } else
        throw("This constraint does not exist");
}

void AnnotationConstraintDialog::slotSave(){
    this->close();

    emit addSemanticRelationship(this->type,
                                 this->ui->weightSpinBox->value(),
                                 this->ui->doubleSpinBox1->value(),
                                 this->ui->doubleSpinBox2->value(),
                                 this->ui->measure1ComboBox->currentIndex(),
                                 this->ui->measure2ComboBox->currentIndex(),
                                 this->directed);
}

std::string AnnotationConstraintDialog::getTypeString() const
{
    return type;
}

std::vector<Annotation *> AnnotationConstraintDialog::getSubjects() const
{
    return subjects;
}

void AnnotationConstraintDialog::setSubjects(const std::vector<Annotation *> &value)
{
    subjects = value;
    this->ui->typeList->clear();
    this->ui->measure1ComboBox->clear();
    this->ui->measure2ComboBox->clear();
    if(value.size() > 0){
        AnnotationType currentType = value[0]->getType();
        if(value.size() == 1){
            this->directed = true;
            if(currentType == AnnotationType::Surface){
                this->ui->typeList->addItem("Surface area");
                this->ui->typeList->addItem("Surface planarity");
                this->ui->typeList->addItem("Surface rigidity");
                this->ui->typeList->addItem("Surface similarity");
                this->ui->typeList->addItem("Surface curvature");
                this->ui->typeList->addItem("Surface length");
                this->ui->typeList->addItem("Surface width");
                this->ui->typeList->addItem("Surface same measure");
                this->ui->typeList->addItem("Surface orientation");
                this->ui->typeList->addItem("Surface regression plane orientation");
                this->ui->typeList->addItem("Surface keeping");
            }else if (currentType == AnnotationType::Line){
                this->ui->typeList->addItem("Line linearity");
                this->ui->typeList->addItem("Line planarity");
                this->ui->typeList->addItem("Line circularity");
                this->ui->typeList->addItem("Line rectangularity");
                this->ui->typeList->addItem("Line angle");
                this->ui->typeList->addItem("Line length");
                this->ui->typeList->addItem("Line safeguard");
                this->ui->typeList->addItem("Line keeping");
            }else if (currentType == AnnotationType::Point) {
                this->ui->typeList->addItem("Point closeness");
                this->ui->typeList->addItem("Point laplacian");
                this->ui->typeList->addItem("Point laplacian displacement");
            }
            for(unsigned int i = 0; i < value[0]->getAttributes().size(); i++)
            {
                std::string s = value[0]->getAttributes()[i]->getKey() + " with id " + std::to_string(value[0]->getAttributes()[i]->getId());
                this->ui->measure1ComboBox->addItem(QString::fromStdString(s));
                this->ui->measure2ComboBox->addItem(QString::fromStdString(s));
            }
        } else {
            for (unsigned int i = 0; i < value.size(); i++)
                if(currentType != value[i]->getType())
                    throw("Constraints between different kinds of annotations are not currently allowed");

            if(currentType == AnnotationType::Surface){
                this->ui->typeList->addItem("Surfaces co-planarity");
                this->ui->typeList->addItem("Surfaces structural continuity");
                this->ui->typeList->addItem("Surfaces same area");
                this->ui->typeList->addItem("Surfaces rigidity");
                this->ui->typeList->addItem("Surfaces same measure");
                this->ui->typeList->addItem("Surfaces same width");
                this->ui->typeList->addItem("Surfaces same orientation");
                this->ui->typeList->addItem("Surfaces angle");
                this->ui->typeList->addItem("Surfaces same level");
                this->ui->typeList->addItem("Surfaces symmetry");
            }else if(currentType == AnnotationType::Line){
                this->ui->typeList->addItem("Lines parallelism");
                this->ui->typeList->addItem("Lines perpendicularity");
                this->ui->typeList->addItem("Lines same length");
                this->ui->typeList->addItem("Lines symmetry");
                this->ui->typeList->addItem("Lines coplanarity");
                this->ui->typeList->addItem("Lines same level");
            } else if(currentType == AnnotationType::Point){
                this->ui->typeList->addItem("Points edge strain");
                this->ui->typeList->addItem("Points triangle strain");
                this->ui->typeList->addItem("Points tetrahedron strain");
                this->ui->typeList->addItem("Points area");
                this->ui->typeList->addItem("Points volume");
                this->ui->typeList->addItem("Points bending");
                this->ui->typeList->addItem("Points line");
                this->ui->typeList->addItem("Points plane");
                this->ui->typeList->addItem("Points circle");
                this->ui->typeList->addItem("Points sphere");
                this->ui->typeList->addItem("Points rigidity");
                this->ui->typeList->addItem("Points similarity");
                this->ui->typeList->addItem("Points rectangle");
                this->ui->typeList->addItem("Points parallelogram");
                this->ui->typeList->addItem("Points angle");
            }

            if(value.size() == 2)
            {

                for(unsigned int i = 0; i < value[0]->getAttributes().size(); i++)
                {
                    std::string s = "\""+value[0]->getAttributes()[i]->getKey() + "\"" + " with id " + std::to_string(value[0]->getAttributes()[i]->getId());
                    this->ui->measure1ComboBox->addItem(QString::fromStdString(s));
                }

                for(unsigned int i = 0; i < value[1]->getAttributes().size(); i++)
                {
                    std::string s = "\""+value[1]->getAttributes()[i]->getKey() + "\"" + " with id " + std::to_string(value[1]->getAttributes()[i]->getId());
                    this->ui->measure2ComboBox->addItem(QString::fromStdString(s));
                }
            }
        }
    }
}
