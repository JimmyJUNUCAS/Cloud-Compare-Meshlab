    const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	//得到实例，选择一个点云阵
	size_t selNum = selectedEntities.size();
	//确定点云阵列的大小
	ccHObject* ent = selectedEntities[0];
	//将点云中选择其中代号0
	ccPointCloud* pc = static_cast<ccPointCloud*>(ent);
	//由实体获得真的点云，将实体按照点云的结构去构建
	//input cloud
	unsigned count = pc->size();
	//点云的大小count
	bool hasNorms = pc->hasNormals();
	//点云是否有法向量
	CCVector3 bbMin, bbMax;
	//声明两个三维向量，boundingbox的最大值点与boundingbox的最小值点
	pc->getBoundingBox(bbMin,bbMax);
	//得到点云的boundingbox
	const CCVector3d& globalShift = pc->getGlobalShift();
	//定义全局变换
	double globalScale = pc->getGlobalScale();
	//全局尺度
	//Convert CC point cloud to RANSAC_SD type
	
	PointCloud cloud;
	{
			//default point & normal
		Point Pt;
		Pt.normal[0] = 0.0;
		Pt.normal[1] = 0.0;
		Pt.normal[2] = 0.0;
		for (unsigned i=0; i<count; ++i)
		{
			const CCVector3* P = pc->getPoint(i);
			Pt.pos[0] = static_cast<float>(P->x);
			Pt.pos[1] = static_cast<float>(P->y);
			Pt.pos[2] = static_cast<float>(P->z);
			if (hasNorms)
			{
				const CCVector3& N = pc->getPointNormal(i);
				Pt.normal[0] = static_cast<float>(N.x);
				Pt.normal[1] = static_cast<float>(N.y);
				Pt.normal[2] = static_cast<float>(N.z);
			}
			cloud.push_back(Pt);
		}
		
		//manually set bounding box!
		Vec3f cbbMin,cbbMax;
		cbbMin[0] = static_cast<float>(bbMin.x);
		cbbMin[1] = static_cast<float>(bbMin.y);
		cbbMin[2] = static_cast<float>(bbMin.z);
		cbbMax[0] = static_cast<float>(bbMax.x);
		cbbMax[1] = static_cast<float>(bbMax.y);
		cbbMax[2] = static_cast<float>(bbMax.z);
		cloud.setBBox(cbbMin,cbbMax);
	}
	//cloud scale (useful for setting several parameters
	const float scale = cloud.getScale();
	//得到云尺度

	//init dialog with default values
	ccRansacSDDlg rsdDlg(m_app->getMainWindow());
	//Right Dialog
	rsdDlg.epsilonDoubleSpinBox->setValue(.005f * scale);
	//Left Dialog
	rsdDlg.planeCheckBox->setChecked(s_primEnabled[0]);
	s_supportPoints = rsdDlg.supportPointsSpinBox->value();
	s_maxNormalDev_deg = rsdDlg.maxNormDevAngleSpinBox->value();
	s_proba = rsdDlg.probaDoubleSpinBox->value();

	plane->Parameters(cloud[count-1-j].pos,&param);



MeshModel *mm = meshDoc()->mm();
	VertexIterator vi = mm->cm.vert.begin();
	for (;vi != mm->cm.vert.end(); vi++)
	{
		(*vi).P()[0];
	}
