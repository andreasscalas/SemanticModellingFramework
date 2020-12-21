#ifndef CONSTRAINTSFILEMANAGER_H
#define CONSTRAINTSFILEMANAGER_H

#include <string>
#include <vector>
#include <filesystem>
#include <iostream>
#include <Constraint.h>
#include <extendedtrimesh.h>
#include <rapidjson/writer.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/reader.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>

class ConstraintsFileManager
{
public:
    static const int BUFFER_SIZE = 65536;

    ConstraintsFileManager();
    bool writeConstraints(std::string fileName);
    bool readConstraints(std::string fileName);

    ExtendedTrimesh *getMesh() const;
    void setMesh(ExtendedTrimesh *value);

    ShapeOp::MatrixX3 *getSoPoints() const;
    void setSoPoints(ShapeOp::MatrixX3 *value);

    std::vector<std::shared_ptr<ShapeOp::Constraint> > getConstraints() const;
    void setConstraints(const std::vector<std::shared_ptr<ShapeOp::Constraint> > &value);

private:
    ExtendedTrimesh* mesh;
    std::vector<std::shared_ptr<ShapeOp::Constraint>> constraints;
    ShapeOp::MatrixX3 *soPoints;
};

#endif // CONSTRAINTSFILEMANAGER_H
