#ifndef ANNOTATIONFILEMANAGER_H
#define ANNOTATIONFILEMANAGER_H

#include <string>
#include <iostream>
#include <extendedtrimesh.h>

class AnnotationFileManager
{
public:
    static const int BUFFER_SIZE = 65536;

    AnnotationFileManager();
    bool writeAnnotations(std::string fileName);
    bool readAnnotations(std::string fileName);

    ExtendedTrimesh *getMesh() const;
    void setMesh(ExtendedTrimesh *value);

private:
    ExtendedTrimesh* mesh;
};

#endif // ANNOTATIONFILEMANAGER_H
