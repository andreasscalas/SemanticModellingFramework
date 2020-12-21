#ifndef VTKGRAPHTOPOLYDATAEXTENDED_H
#define VTKGRAPHTOPOLYDATAEXTENDED_H

#include <vtkFloatArray.h>
#include <vtkGraphToPolyData.h>

class vtkGraphToPolydataExtended : public vtkGraphToPolyData
{
public:
    static vtkGraphToPolydataExtended* New();
    vtkGraphToPolydataExtended();
    vtkTypeMacro(vtkGraphToPolydataExtended, vtkGraphToPolyData)
    virtual int RequestData(
      vtkInformation *vtkNotUsed(request),
      vtkInformationVector **inputVector,
      vtkInformationVector *outputVector) override;

    vtkSmartPointer<vtkFloatArray> getEdgeGlyphPositions() const;
    void setEdgeGlyphPositions(const vtkSmartPointer<vtkFloatArray> &value);

protected:
    vtkSmartPointer<vtkFloatArray> EdgeGlyphPositions;
};

#endif // VTKGRAPHTOPOLYDATAEXTENDED_H
