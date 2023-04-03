#include "annotationdialog.h"
#include "ui_annotationdialog.h"
#include <QColorDialog>
#include <QColor>

AnnotationDialog::AnnotationDialog(QWidget *parent) :
    QDialog(parent), ui(new Ui::AnnotationDialog){
    ui->setupUi(this);
    this->setWindowTitle("AnnotationDialog");
    this->tag = this->ui->tagEdit->text().toStdString();
    connect(this->ui->tagEdit, SIGNAL(textEdited(QString)), this, SLOT(slotTagEdit(QString)));
    connect(this->ui->colorButton, SIGNAL(clicked()), this, SLOT(slotColor()));
    connect(this->ui->ontologyDialogButton, SIGNAL(clicked()), this, SLOT(slotOntologyDialog()));
    connect(this->ui->saveButton, SIGNAL(clicked()), this, SLOT(slotSave()));
}

AnnotationDialog::~AnnotationDialog(){
    delete ui;
}

void AnnotationDialog::slotColor(){
    QColor tmpColor = QColorDialog::getColor();
    int r, g, b;
    tmpColor.getRgb(&r, &g, &b);
    color[0] = static_cast<uchar>(r);
    color[1] = static_cast<uchar>(g);
    color[2] = static_cast<uchar>(b);
    QPalette pal = palette();
    pal.setColor(QPalette::Background, tmpColor);
    this->ui->frame->setAutoFillBackground(true);
    this->ui->frame->setPalette(pal);
    this->ui->frame->show();
}

void AnnotationDialog::slotOntologyDialog(){
    ;
}


void AnnotationDialog::slotTagEdit(QString text){
    tag = text.toStdString();
}

void AnnotationDialog::slotSave(){
    emit finalizationCalled(tag, color);
}
