#ifndef CHECKABLELIST_H
#define CHECKABLELIST_H

#include <QWidget>
#include <QScrollArea>
#include <QCheckBox>
#include <QVBoxLayout>

class CheckableList : public QScrollArea
{
    Q_OBJECT
public:
    explicit CheckableList(QWidget *parent = nullptr);
    void clear();
    QCheckBox* addItem(std::string);
    QCheckBox* addItem(QString);
    void removeItem(QCheckBox*);
private:
    QVBoxLayout* containerLayout;
    std::vector<QCheckBox*> checkboxes;
    void createLayout();
    void createConnections();

};

#endif // CHECKABLELIST_H
