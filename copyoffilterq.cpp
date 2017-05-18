#include "filter_qRANSACSD.h"
#include <QtScript>
#include "PointCloud.h"
#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <ConePrimitiveShapeConstructor.h>
#include <TorusPrimitiveShapeConstructor.h>
#include <PlanePrimitiveShape.h>
#include <SpherePrimitiveShape.h>
#include <CylinderPrimitiveShape.h>
#include <ConePrimitiveShape.h>
#include <TorusPrimitiveShape.h>

#include <QtGui>
#include <QApplication>
#include <QtConcurrentRun>
#include <QApplication>
#include <QProgressDialog>
#include <QMainWindow>
#include <algorithm>
#if defined(CC_WINDOWS)
#include "Windows.h"
#else
#include <time.h>
#endif

// Constructor usually performs only two simple tasks of filling the two lists 
//  - typeList: with all the possible id of the filtering actions
//  - actionList with the corresponding actions. If you want to add icons to your filtering actions you can do here by construction the QActions accordingly

QRanSacSDPlugin::QRanSacSDPlugin() 
{ 
	typeList << FP_QRANSACSD;

  foreach(FilterIDType tt , types())
	  actionList << new QAction(filterName(tt), this);
}

// ST() must return the very short string describing each filtering action 
// (this string is used also to define the menu entry)
QString QRanSacSDPlugin::filterName(FilterIDType filterId) const
{
  switch(filterId) {
  case FP_QRANSACSD: { std::cout << "TEST" << std::endl; return QString("Ransac Shape Detection"); }
		default : assert(0); 
	}
  return QString();
}

// Info() must return the longer string describing each filtering action 
// (this string is used in the About plugin dialog)
 QString QRanSacSDPlugin::filterInfo(FilterIDType filterId) const
{
  switch(filterId) {
		case FP_QRANSACSD :  return QString("Ransac Shape Detection"); 
		default : assert(0); 
	}
	return QString("Unknown Filter");
}

// The FilterClass describes in which generic class of filters it fits. 
// This choice affect the submenu in which each filter will be placed 
// More than a single class can be choosen.
QRanSacSDPlugin::FilterClass QRanSacSDPlugin::getClass(QAction *a)
{
  switch(ID(a))
	{
		case FP_QRANSACSD :  return MeshFilterInterface::Smoothing; 
		default : assert(0); 
	}
	return MeshFilterInterface::Generic;
}

// This function define the needed parameters for each filter. Return true if the filter has some parameters
// it is called every time, so you can set the default value of parameters according to the mesh
// For each parameter you need to define, 
// - the name of the parameter, 
// - the string shown in the dialog 
// - the default value
// - a possibly long string describing the meaning of that parameter (shown as a popup help in the dialog)
void QRanSacSDPlugin::initParameterSet(QAction *action, MeshDocument &md, RichParameterSet & parlst)
{
	 switch(ID(action))	 {
		case FP_QRANSACSD :  
 
		parlst.addParam(new RichBool ("PlaneFlag",true,"PPlane", "The shape of Plane"));
		parlst.addParam(new RichBool ("SphereFlag",true,"Sphere", "The shape of Sphere"));
		parlst.addParam(new RichBool ("CylinderFlag",true,"Cylinder", "The shape of Cylinder"));
		parlst.addParam(new RichBool ("ConeFlag",true,"Cone", "The shape of Cone"));
		parlst.addParam(new RichBool ("TorusFlag",true,"Torus","The shape of Torus"));
	    parlst.addParam(new RichInt  ("SubDelta", 500, "Min support points per primitive",
                                        "This is the minture number of points per primitive"));								
		//parlst.addParam(new RichFloat("Threshold",0.058,"max distance to primitive","Max Distance"));
		parlst.addParam(new RichFloat("epsilon",0.058,"max distance to primitive","max distance"));
		parlst.addParam(new RichFloat("bitmapepsilon",0.117, "sampling resolution","sampling resolution"));
		parlst.addParam(new RichFloat("maxNormDevAngle",25.0, "max normal deviation", "max normal deviation"));
		parlst.addParam(new RichFloat("probaDouble",0.01,"overlooking probability", "overlooking probability"));
		
	    /*parlst.addParam(new RichBool ("UpdateNormals",
											true,
											"Recompute normals",
											"Toggle the recomputation of the normals after the random displacement.\n\n"
											"If disabled the face normals will remains unchanged resulting in a visually pleasant effect."));	    
		*/
		/*parlst.addParam(new RichAbsPerc("Displacement",
												md.cm->bbox.Diag()/100.0f,0.0f,md.cm.bbox.Diag(),
												"Max displacement",
												"The vertex are displaced of a vector whose norm is bounded by this value"));*/
		/*parlst.addParam(new RichAbsPerc("Delta", mindim / 100.0, 0, mindim,
                                            "Spacing between sampling lines",
                                            "This parameter controls the accuracy of the result and the speed of the computation."
                                            "The time and memory needed to perform the operation usually scale as the reciprocal square of this value."
                                            "For optimal results, this value should be at most half the the smallest feature (i.e. the highest frequency) you want to reproduce."));
		*/
		
		
          break;
											
		default : assert(0); 
	}
}
static MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > >* s_shapes; // stores the detected shapes
static size_t s_remainingPoints = 0;
static RansacShapeDetector* s_detector = 0;
static PointCloud* s_cloud = 0;
void doDetection()
{
	if (!s_detector || !s_cloud || !s_shapes)
		return;

	s_remainingPoints = s_detector->Detect(*s_cloud, 0, s_cloud->size(), s_shapes);
}

static unsigned s_supportPoints = 500;	// this is the minimal numer of points required for a primitive
static double   s_maxNormalDev_deg = 25.0;	// maximal normal deviation from ideal shape (in degrees)
static double   s_proba = 0.01;	// probability that no better candidate was overlooked during sampling
//static bool s_primEnabled[5] = { true,true,true,false,false };
// The Real Core Function doing the actual mesh processing.
// Move Vertex of a random quantity
bool QRanSacSDPlugin::applyFilter(QAction *filter, MeshDocument &md, RichParameterSet & par, vcg::CallBackPos *cb)
{
	MeshModel *mm = md.mm();
	unsigned count = mm->cm.VertexNumber();
	PointCloud cloud;
	{
		
		typedef typename CMeshO::VertexIterator VertexIterator;
		//default point & normal
		Point Pt;
		Pt.normal[0] = 0.0;
		Pt.normal[1] = 0.0;
		Pt.normal[2] = 0.0;
		//MeshModel *mm = meshDoc()->mm();

		//bool hasNorms = mm->cm.

		//Vec3f bbMin, bbMax;
		//bbMin = mm->cm.bbox.min.X();

		VertexIterator vi = mm->cm.vert.begin();
		for (; vi != mm->cm.vert.end(); vi++)
		{
			Pt.pos[0] = (*vi).P()[0];
			Pt.pos[1] = (*vi).P()[1];
			Pt.pos[2] = (*vi).P()[2];
			if ((*vi).HasNormal)
			{
				Pt.normal[0] = (*vi).N()[0];
				Pt.normal[0] = (*vi).N()[0];
				Pt.normal[0] = (*vi).N()[0];
			}

			cloud.push_back(Pt);
		}
		//manually set bounding box!
		Vec3f cbbMin, cbbMax;
		cbbMin[0] = mm->cm.bbox.min.X();
		cbbMin[1] = mm->cm.bbox.min.Y();
		cbbMin[2] = mm->cm.bbox.min.X();
		cbbMax[0] = mm->cm.bbox.max.X();
		cbbMax[1] = mm->cm.bbox.max.Y();
		cbbMax[2] = mm->cm.bbox.max.Z();
		cloud.setBBox(cbbMin, cbbMax);
	}
	const float scale = cloud.getScale();

	s_supportPoints = par.getInt("SubDelta");
	s_maxNormalDev_deg = par.getFloat("maxNormDevAngle");
	s_proba = par.getFloat("probaDouble");

	RansacShapeDetector::Options ransacOptions;
	{
		//ransacOptions.m_epsilon = static_cast<float>(rsdDlg.epsilonDoubleSpinBox->value() * 3.0); //internally this threshold is multiplied by 3!
		ransacOptions.m_epsilon = static_cast<float>(par.getFloat("epsilon") * 3.0);
		//ransacOptions.m_bitmapEpsilon = static_cast<float>(rsdDlg.bitmapEpsilonDoubleSpinBox->value());
		ransacOptions.m_bitmapEpsilon = static_cast<float>(par.getFloat("bitmapepsilon"));
		//ransacOptions.m_normalThresh = static_cast<float>(cos(rsdDlg.maxNormDevAngleSpinBox->value() * CC_DEG_TO_RAD));
		ransacOptions.m_normalThresh = static_cast<float>(par.getFloat("maxNormDevAngle"));
		assert(ransacOptions.m_normalThresh >= 0);
		//ransacOptions.m_probability = static_cast<float>(rsdDlg.probaDoubleSpinBox->value());
		ransacOptions.m_probability = static_cast<float>(par.getFloat("probaDouble"));
		//ransacOptions.m_minSupport = static_cast<unsigned>(rsdDlg.supportPointsSpinBox->value());
		ransacOptions.m_minSupport = static_cast<unsigned>(par.getFloat("SubDelta"));
	}
	RansacShapeDetector detector(ransacOptions);
	switch (par.getEnum("Sampling"))
	{
	case 0 : detector.Add(new PlanePrimitiveShapeConstructor());
	case 1 : detector.Add(new SpherePrimitiveShapeConstructor());
	case 2 : detector.Add(new CylinderPrimitiveShapeConstructor());
	case 3 : detector.Add(new ConePrimitiveShapeConstructor());
	case 4 : detector.Add(new TorusPrimitiveShapeConstructor());
	}

	unsigned remaining = count; //count is the size of the pc unsigned count = pc->size(); ccPointCloud* pc = static_cast<ccPointCloud*>(ent);
	typedef std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > DetectedShape;
	MiscLib::Vector< DetectedShape > shapes;
	{
		//progress dialog (Qtconcurrent::run can't be canceled!)
		//QProgressDialog pDlg("Operation in progress (please wait)", QString(), 0, 0, m_app->getMainWindow());
		//QProgressDialog pDlg("Operation in progress (please wait)", QString(), 0, 0);
		//pDlg.setWindowTitle("Ransac Shape Detection");
		//pDlg.show();
		QApplication::processEvents();
		s_detector = &detector;
		s_shapes = &shapes;
		s_cloud = &cloud;
		QFuture<void> future = QtConcurrent::run(doDetection);//多线程的函数

		while (!future.isFinished())
		{
#if defined(CC_WINDOWS)
			::Sleep(500);
#else
			//usleep(500 * 1000);
#endif
			//pDlg.setValue(pDlg.value() + 1);
			QApplication::processEvents();
			remaining = static_cast<unsigned>(s_remainingPoints);

			//pDlg.hide();
			QApplication::processEvents();
		}
	}

	if (shapes.size() > 0)
	{
		for (MiscLib::Vector<DetectedShape>::const_iterator it = shapes.begin(); it != shapes.end(); ++it)
		{
			const PrimitiveShape* shape = it->first;
			unsigned shapePointsCount = static_cast<unsigned>(it->second);
			std::string desc;
			shape->Description(&desc);
		}
	}

	

	//CMeshO &m = md.mm()->cm;
	/*srand(time(NULL)); 
	const float max_displacement =par.getAbsPerc("Displacement");

	for(unsigned int i = 0; i< m.vert.size(); i++){
		 // Typical usage of the callback for showing a nice progress bar in the bottom. 
		 // First parameter is a 0..100 number indicating percentage of completion, the second is an info string.
		  cb(100*i/m.vert.size(), "Randomly Displacing...");

		Scalarm rndax = (Scalarm(2.0*rand())/RAND_MAX - 1.0 ) *max_displacement;
		Scalarm rnday = (Scalarm(2.0*rand())/RAND_MAX - 1.0 ) *max_displacement;
		Scalarm rndaz = (Scalarm(2.0*rand())/RAND_MAX - 1.0 ) *max_displacement;
		m.vert[i].P() += Point3m(rndax,rnday,rndaz);
	}
	
	// Log function dump textual info in the lower part of the MeshLab screen. 
	Log("Successfully displaced %i vertices",m.vn);
	
	// to access to the parameters of the filter dialog simply use the getXXXX function of the FilterParameter Class
	if(par.getBool("UpdateNormals"))	
			vcg::tri::UpdateNormal<CMeshO>::PerVertexNormalizedPerFace(m);
	
	vcg::tri::UpdateBounding<CMeshO>::Box(m);
	*/
  
	return true;
}


//int postCondition(QAction*) const;
int QRanSacSDPlugin::postCondition(QAction* a) const
{
	switch (ID(a)) {
	case FP_QRANSACSD:
	
    return MeshModel::MM_VERTCOLOR;
	}
	return MeshModel::MM_UNKNOWN;
}

MESHLAB_PLUGIN_NAME_EXPORTER(QRanSacSDPlugin)