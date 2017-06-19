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
#include <iostream>
#include <QtGui>
#include <QtTest/QTest>
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//! Conversion factor from degrees to radians
#ifndef CC_DEG_TO_RAD
#define CC_DEG_TO_RAD (M_PI/180.0)
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
  case FP_QRANSACSD: { return QString("Ransac Shape Detection"); }
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

static unsigned s_supportPoints = 500;	// this is the minimal numer of points required for a primitive
static double   s_maxNormalDev_deg = 25.0;	// maximal normal deviation from ideal shape (in degrees)
static double   s_proba = 0.01;	// probability that no better candidate was overlooked during sampling

void QRanSacSDPlugin::initParameterSet(QAction *action, MeshDocument &md, RichParameterSet & parlst)
{
	 switch(ID(action))	 {
		case FP_QRANSACSD :  
 
		parlst.addParam(new RichBool ("PlaneFlag",false,"Plane", "The shape of Plane"));
		parlst.addParam(new RichBool ("SphereFlag", true,"Sphere", "The shape of Sphere"));
		parlst.addParam(new RichBool ("CylinderFlag", false,"Cylinder", "The shape of Cylinder"));
		parlst.addParam(new RichBool ("ConeFlag", false,"Cone", "The shape of Cone"));
		parlst.addParam(new RichBool ("TorusFlag", false,"Torus","The shape of Torus"));

	    parlst.addParam(new RichInt  ("SubDelta", s_supportPoints, "Min support points per primitive",
                                        "This is the minture number of points per primitive"));								
		//parlst.addParam(new RichFloat("Threshold",0.058,"max distance to primitive","Max Distance"));
		//parlst.addParam(new RichFloat("epsilon",0.058,"max distance to primitive","max distance"));
		//parlst.addParam(new RichFloat("bitmapepsilon",0.117, "sampling resolution","sampling resolution"));
		parlst.addParam(new RichFloat("maxNormDevAngle", s_maxNormalDev_deg, "max normal deviation", "max normal deviation"));
		parlst.addParam(new RichFloat("probaDouble", s_proba,"overlooking probability", "overlooking probability"));
		
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

		VertexIterator vi = mm->cm.vert.begin();
		for (; vi != mm->cm.vert.end(); vi++)
		{
			Pt.pos[0] = (*vi).P()[0];
			Pt.pos[1] = (*vi).P()[1];
			Pt.pos[2] = (*vi).P()[2];
			if ((*vi).HasNormal)
			{
				Pt.normal[0] = (*vi).N()[0];
				Pt.normal[1] = (*vi).N()[1];
				Pt.normal[2] = (*vi).N()[2];
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
	std::cout << cloud.getScale() << " " << cloud.size() << std::endl;
	std::cout << "cloudScale..." << cloud.getScale() << std::endl;
	std::cout << "cloudSize..." << cloud.size() << std::endl;
	RansacShapeDetector::Options ransacOptions;
	{
        //internally this threshold is multiplied by 3!
		//ransacOptions.m_epsilon = static_cast<float>(par.getFloat("epsilon") * 3.0);
		ransacOptions.m_epsilon = static_cast<float>((.005f * scale) * 3.0);
		//ransacOptions.m_bitmapEpsilon = static_cast<float>(par.getFloat("bitmapepsilon"));
		ransacOptions.m_bitmapEpsilon = static_cast<float>(.01f * scale);
		//ransacOptions.m_normalThresh = static_cast<float>(par.getFloat("maxNormDevAngle"));
		ransacOptions.m_normalThresh = static_cast<float>(cos(s_maxNormalDev_deg * CC_DEG_TO_RAD));
		//ransacOptions.m_normalThresh = static_cast<float>(cos(par.getFloat("maxNormDevAngle") * CC_DEG_TO_RAD));
		assert(ransacOptions.m_normalThresh >= 0);
		//ransacOptions.m_probability = static_cast<float>(par.getFloat("probaDouble"));
		ransacOptions.m_probability = static_cast<float>(s_proba);
		//ransacOptions.m_probability = static_cast<float>(par.getFloat("probaDouble"));
		//ransacOptions.m_minSupport = static_cast<unsigned>(par.getFloat("SubDelta"));
		ransacOptions.m_minSupport = static_cast<unsigned>(s_supportPoints);
		//ransacOptions.m_minSupport = static_cast<unsigned>(par.getFloat("SubDelta"));

	}

	RansacShapeDetector detector(ransacOptions);

	if (par.getBool("PlaneFlag"))
	{
		detector.Add(new PlanePrimitiveShapeConstructor());
	}
	else if (par.getBool("SphereFlag"))
	{
		detector.Add(new SpherePrimitiveShapeConstructor());
	}
	else if (par.getBool("CylinderFlag"))
	{
		detector.Add(new CylinderPrimitiveShapeConstructor());
	}
	else if (par.getBool("ConeFlag"))
	{
		detector.Add(new ConePrimitiveShapeConstructor());
	}
	else if (par.getBool("TorusFlag"))
	{
		detector.Add(new TorusPrimitiveShapeConstructor());
	}
	else
	{
		return(0);
	}
	
	unsigned remaining = count; 
	typedef std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > DetectedShape;
	MiscLib::Vector< DetectedShape > shapes;
	{
		s_detector = &detector;
		s_shapes = &shapes;
		s_cloud = &cloud;

		//remaining = detector.Detect(cloud, 0, cloud.size(), &shapes);

		QFuture<void> future = QtConcurrent::run(doDetection);//多线程的函数

		while (!future.isFinished())
		{
			QTest::qSleep(500);
			std::cout << "Waiting....\n" << std::endl;
		}
		std::cout << s_remainingPoints << std::endl;
		remaining = static_cast<unsigned>(s_remainingPoints);

	}
	if (remaining == count)
	{
		std::cout << "Segmentation failed..." << std::endl;
	}
	std::cout <<"shapesize..."<< s_shapes->size() << std::endl;
	std::cout <<"cloudsize..."<< s_cloud->size() << std::endl;

	if (s_shapes->size() > 0)
	{
		for (MiscLib::Vector<DetectedShape>::const_iterator it = shapes.begin(); it != shapes.end(); ++it)
		{
			const PrimitiveShape* shape = it->first;
			unsigned shapePointsCount = static_cast<unsigned>(it->second);
			std::string desc;
			shape->Description(&desc);
			std::cout <<"shapenum..."<< shape->Identifier() << std::endl;

			switch (shape->Identifier())
			{
			case 0: //plane
			{
				const PlanePrimitiveShape* plane = static_cast<const PlanePrimitiveShape*>(shape);
				Vec3f G = plane->Internal().getPosition();
				Vec3f N = plane->Internal().getNormal();
				Vec3f X = plane->getXDim();
				Vec3f Y = plane->getYDim();

				//we look for real plane extents
				float minX, maxX, minY, maxY;
				for (unsigned j = 0; j < shapePointsCount; ++j)
				{
					std::pair<float, float> param;
					plane->Parameters(cloud[count - 1 - j].pos, &param);
					if (j != 0)
					{
						if (minX < param.first)
							minX = param.first;
						else if (maxX > param.first)
							maxX = param.first;
						if (minY < param.second)
							minY = param.second;
						else if (maxY > param.second)
							maxY = param.second;
					}
					else
					{
						minX = maxX = param.first;
						minY = maxY = param.second;
					}
				}

				//we recenter plane (as it is not always the case!)
				float dX = maxX - minX;
				float dY = maxY - minY;
				G += X * (minX + dX / 2);
				G += Y * (minY + dY / 2);
				std::cout << "dX..." << dX << std::endl;
				std::cout << "dY..." << dY << std::endl;


			}
			break;

			case 1: //sphere
			{
				const SpherePrimitiveShape* sphere = static_cast<const SpherePrimitiveShape*>(shape);
				float radius = sphere->Internal().Radius();
				std::cout << radius << std::endl;
				Vec3f CC = sphere->Internal().Center();
				std::cout << CC.getValue() << std::endl;

			}
			break;

			case 2: //cylinder
			{
				const CylinderPrimitiveShape* cyl = static_cast<const CylinderPrimitiveShape*>(shape);
				Vec3f G = cyl->Internal().AxisPosition();
				Vec3f N = cyl->Internal().AxisDirection();
				Vec3f X = cyl->Internal().AngularDirection();
				Vec3f Y = N.cross(X);
				float r = cyl->Internal().Radius();
				float hMin = cyl->MinHeight();
				float hMax = cyl->MaxHeight();
				float h = hMax - hMin;
				G += N * (hMin + h / 2);
				std::cout << "r..." << r << std::endl;
				std::cout << "h..." << h << std::endl;
				//we build matrix from these vecctors
				//cylinder primitive
			}
			break;

			case 3:
			{
				const ConePrimitiveShape* cone = static_cast<const ConePrimitiveShape*>(shape);
				Vec3f CC = cone->Internal().Center();
				Vec3f CA = cone->Internal().AxisDirection();
				float alpha = cone->Internal().Angle();

				//compute max height
				Vec3f minP, maxP;
				float minHeight, maxHeight;
				minP = maxP = cloud[0].pos;
				minHeight = maxHeight = cone->Internal().Height(cloud[0].pos);
				for (size_t j = 1; j<shapePointsCount; ++j)
				{
					float h = cone->Internal().Height(cloud[j].pos);
					if (h < minHeight)
					{
						minHeight = h;
						minP = cloud[j].pos;
					}
					else if (h > maxHeight)
					{
						maxHeight = h;
						maxP = cloud[j].pos;
					}

				}

				float minRadius = tan(alpha)*minHeight;
				float maxRadius = tan(alpha)*maxHeight;

				//let's build the cone primitive
				std::cout << "minRadius..." << minRadius << std::endl;
				std::cout << "maxRadius..." << maxRadius << std::endl;
				std::cout << "Heightcut..." << maxHeight - minHeight << std::endl;
			}
			break;
			case 4: //torus
			{
				const TorusPrimitiveShape* torus = static_cast<const TorusPrimitiveShape*>(shape);

			
				
					Vec3f CC = torus->Internal().Center();
					Vec3f CA = torus->Internal().AxisDirection();
					float minRadius = torus->Internal().MinorRadius();
					float maxRadius = torus->Internal().MajorRadius();

					std::cout << "minRadius..." << minRadius << std::endl;
					std::cout << "maxRadius..." << maxRadius << std::endl;

			}
			break;
			}
		}
	}

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