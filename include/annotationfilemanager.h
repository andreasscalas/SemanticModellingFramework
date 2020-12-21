#ifndef ANNOTATIONFILEMANAGER_H
#define ANNOTATIONFILEMANAGER_H

#include <string>
#include <vector>
#include <filesystem>
#include <iostream>
#include <surfaceannotation.h>
#include <lineannotation.h>
#include <pointannotation.h>
#include <extendedtrimesh.h>
#include <facet.h>
#include <annotationutilities.h>
#include <rapidjson/writer.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/reader.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>

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
