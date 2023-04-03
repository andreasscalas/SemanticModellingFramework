#include "constraintsfilemanager.h"
#include <fstream>

#include <rapidjson/writer.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>

using namespace std;

ConstraintsFileManager::ConstraintsFileManager()
{
    mesh = nullptr;
}

bool ConstraintsFileManager::writeConstraints(std::string fileName)
{
    if(mesh != nullptr){
        string extension = fileName.substr(fileName.find_last_of(".") + 1);
        if(extension.compare("cstr") == 0){
            ofstream constraintsFile;
            rapidjson::StringBuffer s;
            rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(s);
            writer.StartObject();
            writer.Key("constraints");
            writer.StartArray();
            for(unsigned int i = 0; i < constraints.size(); i++){
                auto c = constraints[i];
                writer.StartObject();
                writer.Key("type");
                if (dynamic_cast<ShapeOp::EdgeStrainConstraint*>(c.get()) != NULL) writer.String("EdgeStrain");
                if (dynamic_cast<ShapeOp::TriangleStrainConstraint*>(c.get()) != NULL) writer.String("TriangleStrain");
                if (dynamic_cast<ShapeOp::TetrahedronStrainConstraint*>(c.get()) != NULL) writer.String("TetrahedronStrain");
                if (dynamic_cast<ShapeOp::AreaConstraint*>(c.get()) != NULL) writer.String("Area");
                if (dynamic_cast<ShapeOp::VolumeConstraint*>(c.get()) != NULL) writer.String("Volume");
                if (dynamic_cast<ShapeOp::BendingConstraint*>(c.get()) != NULL) writer.String("Bending");
                if (dynamic_cast<ShapeOp::ClosenessConstraint*>(c.get()) != NULL) writer.String("Closeness");
                if (dynamic_cast<ShapeOp::LineConstraint*>(c.get()) != NULL) writer.String("Line");
                if (dynamic_cast<ShapeOp::PlaneConstraint*>(c.get()) != NULL) writer.String("Plane");
                if (dynamic_cast<ShapeOp::CircleConstraint*>(c.get()) != NULL) writer.String("Circle");
                if (dynamic_cast<ShapeOp::SphereConstraint*>(c.get()) != NULL) writer.String("Sphere");
                if (dynamic_cast<ShapeOp::SimilarityConstraint*>(c.get()) != NULL){
                    if(dynamic_cast<ShapeOp::SimilarityConstraint*>(c.get())->isNotRigid())
                        writer.String("Similarity");
                    else
                        writer.String("Rigid");
                }
                if (dynamic_cast<ShapeOp::RectangleConstraint*>(c.get()) != NULL) writer.String("Rectangle");
                if (dynamic_cast<ShapeOp::ParallelogramConstraint*>(c.get()) != NULL) writer.String("Parallelogram");
                if (dynamic_cast<ShapeOp::UniformLaplacianConstraint*>(c.get()) != NULL){
                    if(dynamic_cast<ShapeOp::UniformLaplacianConstraint*>(c.get())->getDisplacement())
                        writer.String("UniformLaplacianDisplacement");
                    else
                        writer.String("UniformLaplacian");
                }
                if (dynamic_cast<ShapeOp::CotangentLaplacianConstraint*>(c.get()) != NULL){
                    if(dynamic_cast<ShapeOp::CotangentLaplacianConstraint*>(c.get())->getDisplacement())
                        writer.String("CotangentLaplacianDisplacement");
                    else
                        writer.String("CotangentLaplacian");
                }
                if (dynamic_cast<ShapeOp::AngleConstraint*>(c.get()) != NULL) writer.String("Angle");
                if (dynamic_cast<ShapeOp::OrientationConstraint*>(c.get()) != NULL) writer.String("Orientation");
                if (dynamic_cast<ShapeOp::RegressionPlaneOrientationConstraint*>(c.get()) != NULL) writer.String("RegressionPlaneOrientation");
                if (dynamic_cast<ShapeOp::PolylineLengthConstraint*>(c.get()) != NULL) writer.String("PolylineLength");
                if (dynamic_cast<ShapeOp::RectangleFitConstraint*>(c.get()) != NULL) writer.String("RectangleFit");
                if (dynamic_cast<ShapeOp::EquilateralConstraint*>(c.get()) != NULL) writer.String("Equilateral");
                writer.Key("weight");
                writer.Double(c->getWeight());
                writer.Key("ids");
                writer.StartArray();
                vector<int> ids = c->getIds();
                for(unsigned int i = 0; i < ids.size(); i++)
                    writer.Int(ids[i]);
                writer.EndArray();
                writer.EndObject();
            }
            writer.EndArray();
            writer.EndObject();
            constraintsFile.open(fileName);
            constraintsFile << s.GetString();
            constraintsFile.close();
        }
    }else
        return false;
    return true;
}

bool ConstraintsFileManager::readConstraints(std::string fileName)
{
    FILE* fp = fopen(fileName.c_str(),"r");
    char buffer[BUFFER_SIZE];
    rapidjson::FileReadStream frs(fp, buffer, sizeof (buffer));

    ShapeOp::Matrix3X mpt = soPoints->transpose();
    rapidjson::Document document;
    if(!(document.ParseStream(frs).HasParseError())){
        if(document.HasMember("constraints") && document["constraints"].IsArray()){
            rapidjson::Value& contraintsList = document["constraints"];
            for (rapidjson::SizeType i = 0; i < contraintsList.Size(); i++) {
                rapidjson::Value& jsonConstraint = contraintsList[i];
                if(jsonConstraint.IsObject()){
                    assert(jsonConstraint.HasMember("type"));
                    assert(jsonConstraint["type"].IsString());
                    string type = jsonConstraint["type"].GetString();
                    vector<int> ids;
                    if(jsonConstraint.HasMember("ids")){
                        rapidjson::Value& jsonIDs = jsonConstraint["ids"];
                        assert(jsonIDs.IsArray());
                        for(unsigned int j = 0; j < jsonIDs.Size(); j++){
                            assert(jsonIDs[j].IsInt());
                            int id = jsonIDs[j].GetInt();
                            if(id < mesh->V.numels())
                                ids.push_back(id);
                        }
                    }

                    assert(jsonConstraint.HasMember("weight"));
                    assert(jsonConstraint["weight"].IsDouble());
                    double weight = jsonConstraint["weight"].GetDouble();
                    auto c = ShapeOp::Constraint::shapeConstraintFactory(type, ids, weight, mpt);
                    this->constraints.push_back(c);
                }else
                    return false;
            }
        }else
            return false;
    }else
        return false;

    return true;

}

ExtendedTrimesh *ConstraintsFileManager::getMesh() const
{
    return mesh;
}

void ConstraintsFileManager::setMesh(ExtendedTrimesh *value)
{
    mesh = value;
}

ShapeOp::MatrixX3 *ConstraintsFileManager::getSoPoints() const
{
    return soPoints;
}

void ConstraintsFileManager::setSoPoints(ShapeOp::MatrixX3 *value)
{
    soPoints = value;
}

std::vector<std::shared_ptr<ShapeOp::Constraint> > ConstraintsFileManager::getConstraints() const
{
    return constraints;
}

void ConstraintsFileManager::setConstraints(const std::vector<std::shared_ptr<ShapeOp::Constraint> > &value)
{
    constraints = value;
}
