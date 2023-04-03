#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//Qt inclusions
#include <QFileDialog>
#include <QMainWindow>
#include <QColorDialog>
#include <QMessageBox>

//Project inclusions
#include <string.h>
#include <drawablemesh.h>
#include <cageverticesselectionstyle.h>
#include <meshdeformationstyle.h>
#include <slicer.h>
#include <slicerpalette.h>
#include <infopalette.h>
#include <layerdialog.h>
#include <Constraint.h>

#include <annotationwindow.h>

//VTK inclusions
#include <vtkPropAssembly.h>
#include <vtkLabeledDataMapper.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    //Macros of the class
    static const uint MVC = 0;                                  //Identifies the Mean Value Coordinates
    static const uint GC = 1;                                   //identifies the Green Coordinates
    const uint SLICES_NUMBER = 100;
    const uint THREADS_NUMBER = 16;
    const uint MAX_CLUSTERS_NUMBER = 10;
    const double MAX_ERROR = 0.05;
    const double MAX_LOD = 100;
    static const int BUFFER_SIZE = 65536;

    //The starting message written in the window.
    const std::string initialInstruction = "All the interactions can be achieved using the toolbar";


    //Contructor and destructor of the class.
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void updateMeshView();

private slots:
    /**
     * @brief slotOpenFile Slot that manages the click on the "folder" button. Loads a mesh.
     */
    void slotOpenFile();

    void slotOpenMesh();
    void slotOpenCage();
    void slotOpenBC();
    void slotOpenConstraints();
    void slotOpenFragment();

    void slotSaveMesh();
    void slotSaveCage();
    void slotSaveBC();
    void slotSaveConstraints();
    /**
     * @brief slotClearAll Slot that manages the click on the "brush" button. Clears the status of the window.
     */
    void slotClearAll();
    void slotClearMesh();
    void slotClearCage();
    void slotClearConstraints();
    void slotClearCamera();
    void slotClearSecondaryMeshes();
    void slotChangeToDeformer();
    void slotChangeToAnnotator();
    void slotChangeToSlicer();
    void slotClose();
    void slotGenerate();
    void slotComputeCoords();
    void slotShowMesh(bool value);
    void slotShowMeshSurface(bool value);
    void slotShowMeshPoints(bool value);
    void slotShowMeshAnnotations(bool);
    void slotShowMeshWireframe(bool value);
    void slotShowCage(bool value);
    void slotShowCageSurface(bool value);
    void slotShowCageWireframe(bool value);
    void slotShowCagePoints(bool value);
    void slotSlicerPalette(bool value);
    void slotInfoPalette(bool value);
    void slotLayerDialog(bool value);
    void slotCheckConstraints();

    /**
     * @brief slotSaveFile Slot that manages the click on the "floppy disk" button. Saves a mesh and, if available, its cage and associated barycentric coordinates.
     */
    void slotSaveFile();


    /**
     * @brief slotModel Slot that manages the click on the "mesh" button. Allows for showing/hiding the model.
     * @param checked true if the model has to be visualized, false otherwise.
     */
    void slotModel(bool checked);

    /**
     * @brief slotCamera Slot that manages the click on the "camera" button. Allows the switching in Camera Mode.
     * @param checked True if the interaction mode have to be set on Camera, false otherwise.
     */
    void slotCamera(bool checked);

    /**
     * @brief slotVerticesSelection Slot that manages the click on the "red vertices with a plus sign" button. Allows the switching in Vertices Selection Mode.
     * @param checked True if the interaction mode have to be set on Vertices Selection, false otherwise.
     */
    void slotVerticesSelection(bool checked);

    /**
     * @brief slotVerticesDeselection Slot that manages the click on the "light blue vertices with a minus sign" button. Allows the switching in Vertices Deselection Mode.
     * @param checked True if the interaction mode have to be set on Vertices Deselection, false otherwise.
     */
    void slotVerticesDeselection(bool checked);

    /**
     * @brief slotVerticesDeformation Slot that manages the click on the "blue and red vertices" button. Allows the switching in Vertices Deformation Mode.
     * @param checked True if the interaction mode have to be set on Vertices Deformation, false otherwise.
     */
    void slotMeshDeformation(bool checked);

    void slotStretch(bool checked);
    /**
     * @brief slotGreen Slot that manages the click on the "G" button. Alows the swtching to the Green Coordinates for deformation.
     */
    void slotGreen();

    /**
     * @brief slotMeanValue Slot that manages the click on the "Mean Value" button. Allows the switching to the Mean Value Coordinates for deformation.
     */
    void slotMeanValue();

    /**
     * @brief slotMeshColor Slot that manages the meshColorChanged event (risen from the color selection popup window. Changes the model color.
     * @param meshColor The new color of the model.
     */
    void slotMeshColor();

    /**
     * @brief slotCageColor Slot that manages the cageColorChanged event (risen from the color selection popup window). Changes the cage color.
     * @param cageColor The new color of the cage.
     */
    void slotCageColor();

    /**
     * @brief slotVisible Slot that manages the click on the "eye" button. Allows the switching between the "visible" and "all" selection mode
     * @param checked
     */
    void slotVisible(bool checked);

    void slotAddAnnotationsConstraint(std::string, double, double, double, unsigned int, unsigned int, bool);

    void slotConstrainRelationship(AnnotationsRelationship*&);

    /**
     * @brief slotConstrain Slot that manages the constrain event (risen from the constraint popup dialog). Allows to impose a constraint onto a set of points.
     * @param type the type of constraint
     * @param weight the importance of the constraint
     */
    void slotConstrain();

    void slotConstrainSystem(std::vector<AnnotationsConstraint*>);
    void slotReleaseSystem();

    void slotSubstituteMesh(DrawableMesh*, DrawableMesh*, QString);

    void on_xSlider_valueChanged(int value);
    void on_ySlider_valueChanged(int value);
    void on_zSlider_valueChanged(int value);
    void on_showNodesBox_stateChanged(int value);
    void on_showArcNodesBox_stateChanged(int value);
    void on_levelOfDetailSlider_valueChanged(int value);
    void on_maxErrorSlider_valueChanged(int value);
    void on_showSlices_stateChanged(int value);
    void on_showFittedCircle_stateChanged(int value);
    void on_showSkeleton_stateChanged(int value);
    void on_showConvexHull_stateChanged(int value);

    void slotUpdateMeshView(DrawableMesh*);
    void slotDeleteMesh(DrawableMesh*);
    void slotEditAnnotations(DrawableMesh*);
    void slotFitRigidly(DrawableMesh*);
    void slotAdaptTemplate(DrawableMesh*);
    void slotAnnotationWindowClosed(DrawableMesh*, std::vector<AnnotationsConstraint*> );

    void slotCurrentPathChanged(std::string);

    void on_xSlider2_valueChanged(int value);
    void on_ySlider2_valueChanged(int value);
    void on_zSlider2_valueChanged(int value);
    void on_showBoundingBox_stateChanged(int value);
    void on_clusterNumberBox_valueChanged(int value);
    void on_showBoundingRectangle_stateChanged(int value);
    void on_showDiagonals_stateChanged(int value);
    void on_showCenters_stateChanged(int value);
    void on_clusterCheckBox_stateChanged(int value);

private:

    //Graphic components
    Ui::MainWindow *ui;
    SlicerPalette *sui;
    InfoPalette *iui;
    LayerDialog *lui;
    AnnotationDialog* ad;
    AnnotationConstraintDialog* cd;
    QVTKInteractor *iren;                                               //Interactor (see QVTKInteractor)
    vtkSmartPointer<vtkRenderer>  ren;                                  //Renderer (see vtkRenderer)
    vtkSmartPointer<vtkRenderer>  sliceRen;
    vtkSmartPointer<vtkPropAssembly>  meshAssembly;                         //Canvas on which all the other canvas are assembled
    vtkSmartPointer<vtkPropAssembly>  visualPropertiesAssembly;
    vtkSmartPointer<vtkPropAssembly>  sliceAssembly;
    vtkSmartPointer<vtkPropAssembly>  measuresAssembly;
    vtkSmartPointer<vtkPropAssembly>  modelCanvas;                          //Canvas on which the model is drawed
    vtkSmartPointer<vtkPropAssembly>  cageCanvas;                           //Canvas on which the cage is drawed
    vtkSmartPointer<vtkActor>     writeCanvas;                          //Canvas on which the write is written
    vtkSmartPointer<vtkCamera>    initialCamera;
    vtkSmartPointer<CageVerticesSelectionStyle> selectionStyle;         //For selecting vertices on the cage.
    std::map<unsigned long, bool> pointsSelectionStatus;                //Data structure for holding the selected points ids.
    vtkSmartPointer<MeshDeformationStyle> deformationStyle;             //For deformating the cage (and consequently the model).
    vtkSmartPointer<vtkPolyData> modelTriangles;                        //VTK data structure for holding the points of the model.
    vtkSmartPointer<vtkPolyData> modelPoints;                           //VTK data structure for holding the points of the model.
    vtkSmartPointer<vtkPolyData> cagePoints;                            //VTK data structure for holding the points of the cage.

    //Useful data structures
    DrawableMesh* model;                                                //The visualized model.
    DrawableMesh* cage;                                                 //The cage associated to the model (optional).
    std::vector<DrawableMesh*> fragments;
    ShapeOpBarycentricCoordinates* soCoords;                            //The barycentric coordinates for the current model and cage.
    ConstraintSolver* solver;
    ShapeOp::MatrixX3* soModelPoints;
    ShapeOp::MatrixX3* soCagePoints;
    std::vector<ShapeOp::MatrixX3*> soFragmentsPoints;
    Slicer* slicer;
    bool newWindow;                                                     //True if the window is newly created.                                                  //True if the window is newly created.
    bool meshLoaded;                                                    //True if the mesh has been loaded.
    bool cageLoaded;                                                    //True if the cage has been loaded.
    bool coordsComputed;
    bool clusterChanged;
    short coordType;                                                    //Holds the type of barycentric coordinates (since now Mean Value and Green).
    bool debugMode;
    bool slicesExtracted;
    bool firstFragmentPositioned;

    Eigen::Matrix4d xRotationMatrix;
    Eigen::Matrix4d yRotationMatrix;
    Eigen::Matrix4d zRotationMatrix;
    Eigen::Vector3d translationVector;
    Eigen::Matrix4d translationMatrix;
    IMATI_STL::Point normal;
    IMATI_STL::Point center;
    int previousXValue;
    int previousYValue;
    int previousZValue;
    int previousHeightValue;
    unsigned int clusterNumber;
    unsigned int reachedContraintId;
    double totalWidth;
    double totalHeight;
    double totalDepth;
    double shift;
    double allowedError;
    double levelOfDetail;
    bool showSliceOnMesh;
    bool showBoundingBox;
    bool showSkeleton;
    bool showNodes;
    bool showArcNodes;
    bool showFittedCircle;
    bool showConvexHull;
    bool showBoundingRectangle;
    bool showDiagonals;
    bool showCenters;
    bool showClusters;
    bool clusteringDone;
    bool constrained;
    bool fitTemplateOnFragments;
    std::vector<double> boundingBox;
    std::vector<Eigen::VectorXd> points;
    std::vector<std::vector<uint> > clusters;
    std::vector<Slice*> clusteringSlices;
    std::vector<std::shared_ptr<ShapeOp::Constraint> > constraints;
    std::vector<AnnotationsConstraint*> semanticConstraints;
    std::vector<unsigned int> closenessConstraintsID;
    std::map<unsigned int, std::vector<unsigned int> > modelToCage;
    std::map<unsigned int, std::vector<unsigned int> > annotationToCage;
    std::string currentPath;
    QString modelFilename;
    QString cageFilename;
    std::vector<QString> fragmentsNames;

    const std::vector<std::vector<double> > colors = {
        {0, 0, 0},          //BLACK
        {0, 0, 1},          //BLUE
        {0, 1, 0},          //GREEN
        {0, 1, 1},          //CYAN
        {1, 0, 0},          //RED
        {1, 0, 1},          //MAGENTA
        {1, 1, 0},          //YELLOW
        {1, 1, 1},          //WHITE
        {0, 0, 0.5},        //DARK BLUE
        {0, 0.5, 0},        //DARK GREEN
        {0, 0.5, 0.5},      //OCEAN BLUE
        {0.5, 0, 0},        //DARK RED
        {0.5, 0, 0.5},      //PURPLE
        {0.5, 0.5, 0},      //GOLD
        {0.5, 0.5, 0.5},    //GRAY
        {1, 0, 0.5},        //PINK
        {1, 0.5, 0},        //ORANGE
        {1, 0.5, 0.5},      //SALMON
        {0, 1, 0.5},        //SPRING GREEN
        {0.5, 1, 0.5},      //PALE GREEN
        {0, 0.5, 1},        //SLATE BLUE
        {0.5, 0.5, 1},      //MEDIUM SLATE BLUE
        {1, 1, 0.5},        //KHAKI
        {1, 0.5, 1},        //HOT PINK
        {0.5, 1, 1},        //TURQUOISE
    };

    const double BORDEAUX[3] = {156.0/255.0, 14.0/255.0, 14.0/255.0};
    const double SLICE_COLOR[3] = {0, 51/255, 0};


    /**
     * @brief init Initializes the window.
     */
    void init();

    /**
     * @brief clear Clears the status of the window.
     */
    void clear();

    /**
     * @brief write Writes a text in the window.
     * @param message
     */
    void write(std::string message);

    void drawInitialTetrahedron();

    void updateView();
    /**
     * @brief computeCoords Starts the computation of the baricentric coordinates for the current model and cage.
     */
    void computeCoords();

    /**
     * @brief meshLoadedPreparation Initialize some components needed for the interaction after the mesh has been loaded
     */
    void meshLoadedPreparation();

    void updateIntersection();
    void clusterSlices();
    void resetCamera();
    void clearAll();
    void clearMesh();
    void clearCage();
    void clearCoordinates();
    void clearSelections();
    void clearConstraints();
    void clearCamera();
    void clearSecondaryMeshes();
    void associateCageVerticesToAnnotations();


};

#endif // MAINWINDOW_H
