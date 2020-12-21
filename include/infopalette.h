#ifndef INFOPALETTE_H
#define INFOPALETTE_H

#include <QWidget>
#include <qstringlistmodel.h>
#include <qstringlist.h>
#include <qlabel.h>
#include <vector>
#include <string>
#include <Constraint.h>

namespace Ui {

class InfoPalette;
}


class InfoPalette : public QWidget
{
    Q_OBJECT

public:

    const int infoHeight = 12;
    explicit InfoPalette(QWidget *parent = nullptr);
    ~InfoPalette();

    void updateInfo();


    std::vector<std::shared_ptr<ShapeOp::Constraint> > getConstraints() const;
    void setConstraints(const std::vector<std::shared_ptr<ShapeOp::Constraint> > &value);

private:
    std::vector<std::shared_ptr<ShapeOp::Constraint> > constraints;
    Ui::InfoPalette *ui;
};

#endif // INFOPALETTE_H
