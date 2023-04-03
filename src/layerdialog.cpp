#include "layerdialog.h"
#include "ui_layerdialog.h"

#include <QMenu>

#include <transparencydialog.h>

LayerDialog::LayerDialog(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LayerDialog)
{
    bcComputed = false;
    ui->setupUi(this);
}

LayerDialog::~LayerDialog()
{
    delete ui;
}

void LayerDialog::addFragment(std::string str, DrawableMesh* mesh)
{
    addFragment(QString::fromStdString(str), mesh);
}

void LayerDialog::addFragment(QString str, DrawableMesh* mesh)
{
    QCheckBox* checkbox = this->ui->fragmentsList->addItem(str);
    checkbox->setChecked(true);
    this->checkbox_mesh_correspondence.insert(std::make_pair(checkbox, mesh));
    checkbox->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(checkbox, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(showContextMenu(const QPoint&)));
    connect(checkbox, SIGNAL(stateChanged(int)), this, SLOT(slotStateChanged(int)));
}

void LayerDialog::removeFragment(DrawableMesh *mesh)
{

    if(checkbox_mesh_correspondence.size() == 0) return;
    std::map<QCheckBox*, DrawableMesh*>::iterator it;

    for(it = checkbox_mesh_correspondence.begin(); it != checkbox_mesh_correspondence.end(); it++)
        if(it->second == mesh)
            break;

    if(it != checkbox_mesh_correspondence.end())
    {
        this->ui->fragmentsList->removeItem(it->first);
        checkbox_mesh_correspondence.erase(it);
    }
}

void LayerDialog::addMainMesh(std::string str, DrawableMesh *mesh)
{
    addMainMesh(QString::fromStdString(str), mesh);
}

void LayerDialog::addMainMesh(QString str, DrawableMesh *mesh)
{
    QCheckBox* checkbox = this->ui->mainMeshList->addItem(str);
    checkbox->setChecked(true);
    this->checkbox_mesh_correspondence.insert(std::make_pair(checkbox, mesh));
    checkbox->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(checkbox, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(showContextMenu(const QPoint&)));
    connect(checkbox, SIGNAL(stateChanged(int)), this, SLOT(slotStateChanged(int)));
}

void LayerDialog::removeMainMesh(DrawableMesh *mesh)
{
    if(checkbox_mesh_correspondence.size() == 0) return;
    std::map<QCheckBox*, DrawableMesh*>::iterator it;
    for(it = checkbox_mesh_correspondence.begin(); it != checkbox_mesh_correspondence.end(); it++)
        if(it->second == mesh)
            break;

    if(it != checkbox_mesh_correspondence.end())
    {
        this->ui->mainMeshList->removeItem(it->first);
        checkbox_mesh_correspondence.erase(it);
    }
}

void LayerDialog::addCage(std::string str, DrawableMesh *mesh)
{
    addCage(QString::fromStdString(str), mesh);
}

void LayerDialog::addCage(QString str, DrawableMesh *mesh)
{
    QCheckBox* checkbox = this->ui->cageList->addItem(str);
    checkbox->setChecked(true);
    this->checkbox_mesh_correspondence.insert(std::make_pair(checkbox, mesh));
    checkbox->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(checkbox, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(showContextMenu(const QPoint&)));
    connect(checkbox, SIGNAL(stateChanged(int)), this, SLOT(slotStateChanged(int)));
}

void LayerDialog::removeCage(DrawableMesh *mesh)
{
    for(std::map<QCheckBox*, DrawableMesh*>::iterator it = checkbox_mesh_correspondence.begin(); it != checkbox_mesh_correspondence.end(); it++)
        if(it->second == mesh)

        {
            this->ui->cageList->removeItem(it->first);
            checkbox_mesh_correspondence.erase(it);
        }
}

bool LayerDialog::getConstraintsImposed() const
{
    return constraintsImposed;
}

void LayerDialog::setConstraintsImposed(bool value)
{
    constraintsImposed = value;
}

bool LayerDialog::getBcComputed() const
{
    return bcComputed;
}

void LayerDialog::setBcComputed(bool value)
{
    bcComputed = value;
}


void LayerDialog::slotStateChanged(int state)
{
    QCheckBox* checkbox = static_cast<QCheckBox*>(QWidget::sender());
    this->checkbox_mesh_correspondence[checkbox]->setDrawable(state);
    emit(updateMeshView(this->checkbox_mesh_correspondence[checkbox]));
}

void LayerDialog::showContextMenu(const QPoint& pos)
{
    QCheckBox* checkbox = static_cast<QCheckBox*>(QWidget::sender());
    QPoint globalPos = checkbox->mapToGlobal(pos);
        // for QAbstractScrollArea and derived classes you would use:
        // QPoint globalPos = myWidget->viewport()->mapToGlobal(pos);

    QMenu contextualMenu;

    {
        QAction* action = new QAction("Show mesh");
        action->setCheckable(true);
        action->setChecked(checkbox->isChecked());
        contextualMenu.addAction(action);
    }
    {
        QAction* action = new QAction("Show surface");
        action->setCheckable(true);
        action->setChecked(checkbox_mesh_correspondence[checkbox]->getDrawSurface());
        contextualMenu.addAction(action);
    }
    {
        QAction* action = new QAction("Show wireframe");
        action->setCheckable(true);
        action->setChecked(checkbox_mesh_correspondence[checkbox]->getDrawWireframe());
        contextualMenu.addAction(action);
    }
    {
        QAction* action = new QAction("Show points");
        action->setCheckable(true);
        action->setChecked(checkbox_mesh_correspondence[checkbox]->getDrawPoints());
        contextualMenu.addAction(action);
    }
    if(!checkbox_mesh_correspondence[checkbox]->getIsCage()){
        QAction* action = new QAction("Edit annotations");
        contextualMenu.addAction(action);
    }
    if(!checkbox_mesh_correspondence[checkbox]->getIsCage()){
        QAction* action = new QAction("Change annotations opacity");
        contextualMenu.addAction(action);
    }
//    if(!checkbox_mesh_correspondence[checkbox]->getIsCage())
//    {
//        QAction* action = new QAction("Open cage");
//        contextualMenu.addAction(action);
//    }
//    if(!checkbox_mesh_correspondence[checkbox]->getIsCage())
//    {
//        QAction* action = new QAction("Remove cage");
//        contextualMenu.addAction(action);
//    }
//    {
//        QAction* action = new QAction("Open annotations");
//        contextualMenu.addAction(action);
//    }
//    {
//        QAction* action = new QAction("Remove annotations");
//        contextualMenu.addAction(action);
//    }
    if(!checkbox_mesh_correspondence[checkbox]->getIsCage()){
        QAction* action = new QAction("Show annotations");
        action->setCheckable(true);
        action->setChecked(checkbox_mesh_correspondence[checkbox]->getDrawAnnotations());
        if(checkbox_mesh_correspondence[checkbox]->getAnnotations().size() == 0)
            action->setEnabled(false);
        contextualMenu.addAction(action);
    }
    {
    contextualMenu.addAction("Delete mesh");
    }
    if(!checkbox_mesh_correspondence[checkbox]->getIsTemplate() &&
       !checkbox_mesh_correspondence[checkbox]->getIsCage()){
        QAction* action = new QAction("Align rigidly template to fragment");
        contextualMenu.addAction(action);
    }
    if(!checkbox_mesh_correspondence[checkbox]->getIsTemplate() &&
       !checkbox_mesh_correspondence[checkbox]->getIsCage()){
        QAction* action = new QAction("Adapt template to fragment");
        action->setEnabled(bcComputed);
        contextualMenu.addAction(action);
    }

    QAction* selectedItem = contextualMenu.exec(globalPos);
    if(selectedItem != nullptr){
        if(selectedItem->text().compare("Delete mesh") == 0){
            DrawableMesh* mesh = checkbox_mesh_correspondence[checkbox];
            if(mesh->getIsTemplate())
                this->ui->mainMeshList->removeItem(checkbox);
            else if(mesh->getIsCage())
                this->ui->cageList->removeItem(checkbox);
            else
                this->ui->fragmentsList->removeItem(checkbox);
            checkbox_mesh_correspondence.erase(checkbox_mesh_correspondence.find(checkbox));
            emit(deleteMesh(mesh));
        } else if(selectedItem->text().compare("Show mesh") == 0){
            checkbox->setChecked(!checkbox->isChecked());
        } else if(selectedItem->text().compare("Show surface") == 0){
            this->checkbox_mesh_correspondence[checkbox]->setDrawSurface(selectedItem->isChecked());
            emit(updateMeshView(this->checkbox_mesh_correspondence[checkbox]));
        } else if(selectedItem->text().compare("Show wireframe") == 0){
            this->checkbox_mesh_correspondence[checkbox]->setDrawWireframe(selectedItem->isChecked());
            emit(updateMeshView(this->checkbox_mesh_correspondence[checkbox]));
        } else if(selectedItem->text().compare("Show points") == 0){
            this->checkbox_mesh_correspondence[checkbox]->setDrawPoints(selectedItem->isChecked());
            emit(updateMeshView(this->checkbox_mesh_correspondence[checkbox]));
        } else if(selectedItem->text().compare("Open cage") == 0){
            emit(openCage(this->checkbox_mesh_correspondence[checkbox]));
        } else if(selectedItem->text().compare("Remove cage") == 0){
            emit(deleteCage(this->checkbox_mesh_correspondence[checkbox]));
        } else if(selectedItem->text().compare("Open annotations") == 0){
            emit(openAnnotations(this->checkbox_mesh_correspondence[checkbox]));
        } else if(selectedItem->text().compare("Remove annotations") == 0){
            emit(removeAnnotations(this->checkbox_mesh_correspondence[checkbox]));
        } else if(selectedItem->text().compare("Show annotations") == 0){
            this->checkbox_mesh_correspondence[checkbox]->setDrawAnnotations(selectedItem->isChecked());
            emit(updateMeshView(this->checkbox_mesh_correspondence[checkbox]));
        } else if(selectedItem->text().compare("Edit annotations") == 0){
            emit editAnnotations(this->checkbox_mesh_correspondence[checkbox]);
        } else if(selectedItem->text().compare("Change annotations opacity") == 0){
            TransparencyDialog* d = new TransparencyDialog();
            d->setMesh(this->checkbox_mesh_correspondence[checkbox]);
            connect(d, SIGNAL(updateView(DrawableMesh*)), this, SLOT(slotUpdateView(DrawableMesh*)));
            d->show();
        } else if(selectedItem->text().compare("Align rigidly template to fragment") == 0){
            emit(fitRigidly(this->checkbox_mesh_correspondence[checkbox]));
        } else if(selectedItem->text().compare("Adapt template to fragment") == 0){
            emit(adaptTemplate(this->checkbox_mesh_correspondence[checkbox]));
        }
    }

}

void LayerDialog::slotUpdateView(DrawableMesh * mesh)
{

    emit(updateMeshView(mesh));
}
