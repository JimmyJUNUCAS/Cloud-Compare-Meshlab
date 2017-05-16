	//init dialog with default values
	ccRansacSDDlg rsdDlg(m_app->getMainWindow());
	//Right Dialog
	rsdDlg.epsilonDoubleSpinBox->setValue(.005f * scale);		// set distance threshold to 0.5% of bounding box width
	rsdDlg.bitmapEpsilonDoubleSpinBox->setValue(.01f * scale);	// set bitmap resolution (= sampling resolution) to 1% of bounding box width
	rsdDlg.supportPointsSpinBox->setValue(s_supportPoints);
	rsdDlg.maxNormDevAngleSpinBox->setValue(s_maxNormalDev_deg);
	rsdDlg.probaDoubleSpinBox->setValue(s_proba);
	//Left Dialog
	rsdDlg.planeCheckBox->setChecked(s_primEnabled[0]);
	rsdDlg.sphereCheckBox->setChecked(s_primEnabled[1]);
	rsdDlg.cylinderCheckBox->setChecked(s_primEnabled[2]);
	rsdDlg.coneCheckBox->setChecked(s_primEnabled[3]);
	rsdDlg.torusCheckBox->setChecked(s_primEnabled[4]);

	s_supportPoints = rsdDlg.supportPointsSpinBox->value();
	s_maxNormalDev_deg = rsdDlg.maxNormDevAngleSpinBox->value();
	s_proba = rsdDlg.probaDoubleSpinBox->value();


	//MESHLAB
	switch(par.getEnum("Sampling"))
	//得到抽样的类型,使用了enum类型，
    {
    case 0 :	tri::SurfaceSampling<CMeshO,BaseSampler>::VertexUniform(curMM->cm,mps,par.getInt("SampleNum"));	break;
    case 1 :	tri::SurfaceSampling<CMeshO,BaseSampler>::EdgeUniform(curMM->cm,mps,par.getInt("SampleNum"),true); break;
    case 2 :	tri::SurfaceSampling<CMeshO,BaseSampler>::AllFace(curMM->cm,mps); break;
    }
    vcg::tri::UpdateBounding<CMeshO>::Box(mm->cm);
    Log("Sampling created a new mesh of %i points",md.mm()->cm.vn);


    MeshModel *mm= md.addNewMesh("","Sampled Mesh",true/*,rm*/); // After Adding a mesh to a MeshDocument the new mesh is the current one
    bool RecoverColor = par.getBool("RecoverColor");
    BaseSampler mps(&(mm->cm));
    mps.texSamplingWidth=par.getInt("TextureW");
    mps.texSamplingHeight=par.getInt("TextureH");


    if(par.getBool("EdgeSampling"))
    {
      tri::SurfaceSampling<CMeshO,BaseSampler>::EdgeMontecarlo(curMM->cm,mps,par.getInt("SampleNum"),false);
    }
    else
    {
      if(par.getBool("Weighted"))
        tri::SurfaceSampling<CMeshO,BaseSampler>::WeightedMontecarlo(curMM->cm,mps,par.getInt("SampleNum"),par.getFloat("RadiusVariance"));
      else if(par.getBool("ExactNum")) tri::SurfaceSampling<CMeshO,BaseSampler>::Montecarlo(curMM->cm,mps,par.getInt("SampleNum"));
      else tri::SurfaceSampling<CMeshO,BaseSampler>::MontecarloPoisson(curMM->cm,mps,par.getInt("SampleNum"));
    }

    vcg::tri::UpdateBounding<CMeshO>::Box(mm->cm);
    Log("Sampling created a new mesh of %i points",md.mm()->cm.vn);