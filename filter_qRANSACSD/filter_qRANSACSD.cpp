#include "filter_qRANSACSD.h"
#include <QtScript>

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
 
		parlst.addParam(new RichBool ("PlaneFlag",true,"Plane", "The shape of Plane"));
		parlst.addParam(new RichBool ("SphereFlag",true,"Sphere", "The shape of Sphere"));
		parlst.addParam(new RichBool ("CylinderFlag",true,"Cylinder", "The shape of Cylinder"));
		parlst.addParam(new RichBool ("ConeFlag",true,"Cone", "The shape of Cone"));
		parlst.addParam(new RichBool ("TorusFlag",true,"Torus","The shape of Torus"));
	    parlst.addParam(new RichInt  ("SubDelta", 500, "Min support points per promitive",
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

// The Real Core Function doing the actual mesh processing.
// Move Vertex of a random quantity
bool QRanSacSDPlugin::applyFilter(QAction *filter, MeshDocument &md, RichParameterSet & par, vcg::CallBackPos *cb)
{
	//MeshModel *mm = meshDoc()->mm();
    MeshModel *mm = md.mm();
    VertexIterator vi = mm->cm.vert.begin();
    for (;vi != mm->cm.vert.end(); vi++)
    {
        (*vi).P()[0];
    }

    PointCloud cloud;
	{
		try
		{
			cloud.reserve(count);
		}
		catch(...)
		{
			m_app->dispToConsole("Not enough memory!",ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		//default point & normal
		Point Pt;
		Pt.normal[0] = 0.0;
		Pt.normal[1] = 0.0;
		Pt.normal[2] = 0.0;

		 MeshModel *mm = md.mm();
    	 VertexIterator vi = mm->cm.vert.begin();
    	 for (;vi != mm->cm.vert.end(); vi++)
   		 {
        	Pt.pos[0] = static_cast<float> (*vi).P()[0];
        	Pt.pos[1] = static_cast<float> (*vi).P()[1];
        	Pt.pos[2] = static_cast<float> (*vi).P()[2];
    	 }
		for (unsigned i=0; i<count; ++i)
		{
			const CCVector3* P = pc->getPoint(i);
			//得到点云中的所有点，利用三维向量来进行存储
			Pt.pos[0] = static_cast<float>(P->x);
			Pt.pos[1] = static_cast<float>(P->y);
			Pt.pos[2] = static_cast<float>(P->z);
			//如果存在法向量，则把此法向量存储在点云之中
			if (hasNorms)
			{
				const CCVector3& N = pc->getPointNormal(i);
				Pt.normal[0] = static_cast<float>(N.x);
				Pt.normal[1] = static_cast<float>(N.y);
				Pt.normal[2] = static_cast<float>(N.z);
			}
			cloud.push_back(Pt);
			//将此法向量存储在PT之后
		}
		
		//manually set bounding box!
		Vec3f cbbMin,cbbMax;
		//在声明之后，对变量进行赋值
		cbbMin[0] = static_cast<float>(bbMin.x);
		cbbMin[1] = static_cast<float>(bbMin.y);
		cbbMin[2] = static_cast<float>(bbMin.z);
		cbbMax[0] = static_cast<float>(bbMax.x);
		cbbMax[1] = static_cast<float>(bbMax.y);
		cbbMax[2] = static_cast<float>(bbMax.z);
		//针对点云手动设定boundingbox
		cloud.setBBox(cbbMin,cbbMax);
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